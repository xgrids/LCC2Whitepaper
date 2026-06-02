# .btree File Format Specification (LCC2 BVH Binary Tree)

> Version: 1.0  
> Last Updated: 2026-06-02  
> Purpose: Complete format description for third-party developers implementing a .btree Reader

---

## 1. Overview

A `.btree` file is a binary BVH (Bounding Volume Hierarchy) file used for collision detection acceleration within the LCC2 format. Each file stores one complete BVH tree that accelerates spatial queries (ray intersection, collision detection, etc.) against a corresponding triangle mesh (`.ply`).

### Key Properties

| Property | Value |
|----------|-------|
| File Extension | `.btree` |
| Byte Order | **Little-Endian** |
| File Header | **None** (no magic number, no version, no length prefix) |
| Node Size | Fixed **32 bytes** |
| Tree Layout | Depth-first, left-child-first (DFS left-first) |
| Alignment | 32-byte aligned (suitable for direct GPU read) |

---

## 2. File Structure

```
┌─────────────────────────────────────┐  Byte 0
│         Node 0 (Root)               │  32 bytes
├─────────────────────────────────────┤  Byte 32
│         Node 1                      │  32 bytes
├─────────────────────────────────────┤  Byte 64
│         Node 2                      │  32 bytes
├─────────────────────────────────────┤
│              ...                    │
├─────────────────────────────────────┤  Byte (N-1)*32
│         Node N-1                    │  32 bytes
└─────────────────────────────────────┘
```

**Computing node count:**

```
node_count = file_size_in_bytes / 32
```

**Validation:** File size must be a multiple of 32; otherwise the file is corrupted.

---

## 3. Node Binary Layout (32 Bytes)

```
Byte Offset    Size    Type       Field Name         Description
─────────────────────────────────────────────────────────────────────
 0             4       float32    bounds[0]          AABB min X
 4             4       float32    bounds[1]          AABB min Y
 8             4       float32    bounds[2]          AABB min Z
12             4       float32    bounds[3]          AABB max X
16             4       float32    bounds[4]          AABB max Y
20             4       float32    bounds[5]          AABB max Z
24             4       uint32     field6             (meaning depends on node type, see below)
28             4       —          field7             (meaning depends on node type, see below)
─────────────────────────────────────────────────────────────────────
Total: 32 bytes
```

### 3.1 Interpretation of field6 and field7

Nodes are classified as either **Internal Nodes** or **Leaf Nodes**, distinguished by the upper 16 bits of field7:

#### Classification Rule

```
Interpret the 4 bytes of field7 as uint16[2] (Little-Endian):
  uint16_low  = bytes[28..29]   (offset 28-29)
  uint16_high = bytes[30..31]   (offset 30-31)

if (uint16_high == 0xFFFF):
    → Leaf Node
else:
    → Internal Node
```

#### Internal Node

| Field | Type | Description |
|-------|------|-------------|
| field6 | uint32 | Right child **4-byte-unit offset** (right child byte offset = field6 × 4) |
| field7 | uint32 | Split axis (splitAxis): 0=X, 1=Y, 2=Z |

#### Leaf Node

| Field | Type | Description |
|-------|------|-------------|
| field6 | uint32 | Triangle start index (triangleOffset), indexes into the corresponding .ply face array |
| field7 low 16 bits | uint16 | Triangle count (triangleCount) |
| field7 high 16 bits | uint16 | Fixed value `0xFFFF` (leaf marker) |

---

## 4. Tree Topology

### 4.1 Root Node

The root node is **always the first node** in the file, at byte offset = 0.

### 4.2 Parent-Child Relationships

```
For an internal node at byte offset P:

  Left child byte offset  = P + 32           (immediately follows parent)
  Right child byte offset = field6 × 4       (specified by field6)
```

### 4.3 Layout Diagram (DFS Left-First)

```
Index: [0]      [1]      [2]      [3]      [4]      [5]      [6]
       Root     Left     LL       LR       Right    RL       RR
       (int.)   (int.)   (leaf)   (leaf)   (int.)   (leaf)   (leaf)

Example (7 nodes, file size = 224 bytes):
  Root  at byte   0, field6 = 32  → right child at byte 128 (32×4)
  Left  at byte  32, field6 = 24  → right child at byte  96 (24×4)
  LL    at byte  64 (leaf)
  LR    at byte  96 (leaf)
  Right at byte 128, field6 = 48  → right child at byte 192 (48×4)
  RL    at byte 160 (leaf)
  RR    at byte 192 (leaf)
```

The formula is always:

```
right_child_byte_offset = node.field6 * 4
```

---

## 5. BVH Construction Parameters (For Understanding File Content)

| Parameter | Value | Description |
|-----------|-------|-------------|
| MaxDepth | 25 | Maximum tree depth |
| MaxLeafTris | 20 | Maximum triangles per leaf node |
| Split Strategy | Centroid midpoint split | Splits along the longest axis of the centroid bounding box at midpoint |
| Face Reordering | Yes | BVH construction reorders triangle faces; leaf offset/count references the reordered indices |

---

## 6. Relationship with Triangle Mesh

Each `.btree` file is paired with a same-named `.ply` file:

```
mesh/node_0.ply    ← Triangle mesh (vertices + faces)
mesh/node_0.btree  ← BVH acceleration structure
```

**Important:** The BVH construction process **reorders** the triangle faces in the `.ply` file. The `triangleOffset` and `triangleCount` in leaf nodes refer to index ranges in the **reordered** face array. The `.ply` file stores faces in an order consistent with the BVH.

When querying:
```
triangles_in_leaf = ply.faces[triangleOffset .. triangleOffset + triangleCount)
```

---

## 7. Locating .btree Files in meta.lcc2

`meta.lcc2` is a JSON metadata file:

```json
{
  "root": {
    "bvhFiles": ["mesh/node_0.btree", "mesh/node_1.btree"],
    "meshFiles": ["mesh/node_0.ply", "mesh/node_1.ply"],
    "child": {
      "node_0": {
        "data": {
          "bvh": { "name": 0 },
          "mesh": { "name": 0, "vertex": 300, "face": 100 }
        }
      }
    }
  }
}
```

- `bvhFiles[i]`: Path to the i-th BVH file (relative to meta.lcc2 directory)
- Node `data.bvh.name`: BVH file index used by this node (index into the bvhFiles array)
- `data.mesh.name`: Corresponding mesh file index (index into meshFiles array), used together with BVH

---

## 8. Reader Pseudocode

### 8.1 Loading the File

```
function ReadBTreeFile(filepath):
    data = read_all_bytes(filepath)
    
    if data.length == 0:
        error("Empty file")
    
    if data.length % 32 != 0:
        error("Invalid file: size not multiple of 32")
    
    node_count = data.length / 32
    return data, node_count
```

### 8.2 Parsing a Single Node

```
function ParseNode(data, byte_offset):
    node = {}
    
    // Read AABB (6 × float32, Little-Endian)
    node.minX = read_float32_le(data, byte_offset + 0)
    node.minY = read_float32_le(data, byte_offset + 4)
    node.minZ = read_float32_le(data, byte_offset + 8)
    node.maxX = read_float32_le(data, byte_offset + 12)
    node.maxY = read_float32_le(data, byte_offset + 16)
    node.maxZ = read_float32_le(data, byte_offset + 20)
    
    // Read field6 (uint32)
    field6 = read_uint32_le(data, byte_offset + 24)
    
    // Read high/low 16 bits of field7
    uint16_low  = read_uint16_le(data, byte_offset + 28)
    uint16_high = read_uint16_le(data, byte_offset + 30)
    
    // Determine node type
    if uint16_high == 0xFFFF:
        node.is_leaf = true
        node.triangle_offset = field6
        node.triangle_count  = uint16_low
    else:
        node.is_leaf = false
        node.right_child_byte_offset = field6 * 4
        node.split_axis = read_uint32_le(data, byte_offset + 28)
    
    return node
```

### 8.3 Recursive Tree Traversal

```
function TraverseBVH(data, byte_offset):
    if byte_offset >= data.length:
        return
    
    node = ParseNode(data, byte_offset)
    
    if node.is_leaf:
        // Process leaf: access faces[triangle_offset .. triangle_offset + triangle_count)
        process_leaf(node)
    else:
        // Left child immediately follows current node
        TraverseBVH(data, byte_offset + 32)
        // Right child indexed by field6
        TraverseBVH(data, node.right_child_byte_offset)
```

### 8.4 Stack-Based Iterative Traversal (Recommended for Ray Queries)

```
function StackTraverseBVH(data):
    stack = [0]  // Start from root node (byte_offset=0)
    
    while stack is not empty:
        offset = stack.pop()
        if offset >= data.length:
            continue
        
        node = ParseNode(data, offset)
        
        if not intersects(ray, node.aabb):
            continue
        
        if node.is_leaf:
            // Perform exact intersection test against triangles in leaf
            for i in range(node.triangle_offset, node.triangle_offset + node.triangle_count):
                test_ray_triangle(ray, faces[i])
        else:
            // Push right child first (processed later), then left child (processed first)
            stack.push(node.right_child_byte_offset)
            stack.push(offset + 32)
```

---

## 9. Language-Specific Reader Implementations

### 9.1 C / C++

```c
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BTREE_NODE_SIZE 32

typedef struct {
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    int is_leaf;
    // Leaf fields
    uint32_t tri_offset;
    uint16_t tri_count;
    // Internal fields
    uint32_t right_child_byte_offset;
    uint32_t split_axis;
} BTreeNode;

uint8_t* btree_load(const char* path, uint32_t* out_node_count) {
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;
    
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    if (size <= 0 || size % BTREE_NODE_SIZE != 0) {
        fclose(f);
        return NULL;
    }
    
    uint8_t* data = (uint8_t*)malloc(size);
    fread(data, 1, size, f);
    fclose(f);
    
    *out_node_count = (uint32_t)(size / BTREE_NODE_SIZE);
    return data;
}

BTreeNode btree_parse_node(const uint8_t* data, uint32_t byte_offset) {
    BTreeNode node;
    const float*    fp  = (const float*)(data + byte_offset);
    const uint32_t* u32 = (const uint32_t*)(data + byte_offset);
    const uint16_t* u16 = (const uint16_t*)(data + byte_offset);
    
    node.min_x = fp[0]; node.min_y = fp[1]; node.min_z = fp[2];
    node.max_x = fp[3]; node.max_y = fp[4]; node.max_z = fp[5];
    
    node.is_leaf = (u16[15] == 0xFFFF);
    
    if (node.is_leaf) {
        node.tri_offset = u32[6];
        node.tri_count  = u16[14];
        node.right_child_byte_offset = 0;
        node.split_axis = 0;
    } else {
        node.right_child_byte_offset = u32[6] * 4;
        node.split_axis = u32[7];
        node.tri_offset = 0;
        node.tri_count  = 0;
    }
    
    return node;
}
```

### 9.2 Python

```python
import struct
from dataclasses import dataclass

BTREE_NODE_SIZE = 32

@dataclass
class BTreeNode:
    min_x: float; min_y: float; min_z: float
    max_x: float; max_y: float; max_z: float
    is_leaf: bool
    # Leaf fields
    tri_offset: int = 0
    tri_count: int = 0
    # Internal fields
    right_child_byte_offset: int = 0
    split_axis: int = 0

def load_btree(filepath: str) -> bytes:
    with open(filepath, 'rb') as f:
        data = f.read()
    assert len(data) > 0 and len(data) % BTREE_NODE_SIZE == 0, "Invalid .btree file"
    return data

def parse_node(data: bytes, byte_offset: int) -> BTreeNode:
    # Unpack 6 floats + 2 uint32
    values = struct.unpack_from('<6f2I', data, byte_offset)
    min_x, min_y, min_z, max_x, max_y, max_z = values[0:6]
    field6, field7 = values[6], values[7]
    
    # Check leaf marker: high 16 bits of field7
    uint16_high = (field7 >> 16) & 0xFFFF
    
    if uint16_high == 0xFFFF:
        return BTreeNode(
            min_x=min_x, min_y=min_y, min_z=min_z,
            max_x=max_x, max_y=max_y, max_z=max_z,
            is_leaf=True,
            tri_offset=field6,
            tri_count=field7 & 0xFFFF  # low 16 bits
        )
    else:
        return BTreeNode(
            min_x=min_x, min_y=min_y, min_z=min_z,
            max_x=max_x, max_y=max_y, max_z=max_z,
            is_leaf=False,
            right_child_byte_offset=field6 * 4,
            split_axis=field7
        )

def traverse(data: bytes, byte_offset: int = 0, depth: int = 0):
    if byte_offset >= len(data):
        return
    node = parse_node(data, byte_offset)
    indent = "  " * depth
    if node.is_leaf:
        print(f"{indent}[Leaf] tris=[{node.tri_offset}..{node.tri_offset + node.tri_count})")
    else:
        print(f"{indent}[Internal] axis={node.split_axis}")
        traverse(data, byte_offset + BTREE_NODE_SIZE, depth + 1)
        traverse(data, node.right_child_byte_offset, depth + 1)
```

### 9.3 TypeScript / JavaScript

```typescript
const BTREE_NODE_SIZE = 32;

interface BTreeNode {
  minX: number; minY: number; minZ: number;
  maxX: number; maxY: number; maxZ: number;
  isLeaf: boolean;
  triOffset: number;
  triCount: number;
  rightChildByteOffset: number;
  splitAxis: number;
}

function loadBTree(buffer: ArrayBuffer): DataView {
  if (buffer.byteLength === 0 || buffer.byteLength % BTREE_NODE_SIZE !== 0) {
    throw new Error("Invalid .btree file");
  }
  return new DataView(buffer);
}

function parseNode(view: DataView, byteOffset: number): BTreeNode {
  const minX = view.getFloat32(byteOffset + 0, true);  // true = little-endian
  const minY = view.getFloat32(byteOffset + 4, true);
  const minZ = view.getFloat32(byteOffset + 8, true);
  const maxX = view.getFloat32(byteOffset + 12, true);
  const maxY = view.getFloat32(byteOffset + 16, true);
  const maxZ = view.getFloat32(byteOffset + 20, true);
  
  const field6 = view.getUint32(byteOffset + 24, true);
  const uint16Low  = view.getUint16(byteOffset + 28, true);
  const uint16High = view.getUint16(byteOffset + 30, true);
  
  if (uint16High === 0xFFFF) {
    return {
      minX, minY, minZ, maxX, maxY, maxZ,
      isLeaf: true,
      triOffset: field6,
      triCount: uint16Low,
      rightChildByteOffset: 0,
      splitAxis: 0,
    };
  } else {
    return {
      minX, minY, minZ, maxX, maxY, maxZ,
      isLeaf: false,
      triOffset: 0,
      triCount: 0,
      rightChildByteOffset: field6 * 4,
      splitAxis: view.getUint32(byteOffset + 28, true),
    };
  }
}
```

### 9.4 C# / Unity

```csharp
using System;
using System.IO;

public static class BTreeReader
{
    public const int NodeSize = 32;
    
    public struct BTreeNode
    {
        public float MinX, MinY, MinZ, MaxX, MaxY, MaxZ;
        public bool IsLeaf;
        public uint TriOffset, TriCount;
        public uint RightChildByteOffset, SplitAxis;
    }
    
    public static byte[] Load(string path)
    {
        byte[] data = File.ReadAllBytes(path);
        if (data.Length == 0 || data.Length % NodeSize != 0)
            throw new InvalidDataException("Invalid .btree file");
        return data;
    }
    
    public static BTreeNode Parse(byte[] data, int byteOffset)
    {
        float minX = BitConverter.ToSingle(data, byteOffset + 0);
        float minY = BitConverter.ToSingle(data, byteOffset + 4);
        float minZ = BitConverter.ToSingle(data, byteOffset + 8);
        float maxX = BitConverter.ToSingle(data, byteOffset + 12);
        float maxY = BitConverter.ToSingle(data, byteOffset + 16);
        float maxZ = BitConverter.ToSingle(data, byteOffset + 20);
        
        uint field6 = BitConverter.ToUInt32(data, byteOffset + 24);
        ushort u16Low  = BitConverter.ToUInt16(data, byteOffset + 28);
        ushort u16High = BitConverter.ToUInt16(data, byteOffset + 30);
        
        var node = new BTreeNode { MinX = minX, MinY = minY, MinZ = minZ,
                                   MaxX = maxX, MaxY = maxY, MaxZ = maxZ };
        
        if (u16High == 0xFFFF)
        {
            node.IsLeaf = true;
            node.TriOffset = field6;
            node.TriCount = u16Low;
        }
        else
        {
            node.IsLeaf = false;
            node.RightChildByteOffset = field6 * 4;
            node.SplitAxis = BitConverter.ToUInt32(data, byteOffset + 28);
        }
        
        return node;
    }
}
```

---

## 10. Edge Cases and Notes

| Scenario | Behavior |
|----------|----------|
| Empty mesh (0 faces) | No .btree file is generated |
| Single triangle | Only one leaf node (root is leaf), file size = 32 bytes |
| splitAxis value range | Only 0, 1, 2 are valid |
| triangleCount upper bound | uint16 max is 65535 (0xFFFF); in practice constrained by MaxLeafTris=20 |
| triangleOffset | Zero-based, indexes into the reordered face array |
| Right child offset validity | `field6 * 4` must be < file size and a multiple of 32 |
| Float special values | AABB values are standard IEEE 754 float32; may contain very large/small values but should not be NaN/Inf |

---

## 11. Validation Checklist

Recommended checks when implementing a Reader:

- [ ] File size > 0
- [ ] File size % 32 == 0
- [ ] Root node (offset=0) AABB encloses all child node AABBs
- [ ] For internal nodes: `field6 * 4` < file size
- [ ] For internal nodes: `field6 * 4` % 32 == 0
- [ ] For leaf nodes: `triangleCount` > 0
- [ ] splitAxis ∈ {0, 1, 2}
- [ ] Total nodes visited during traversal == file_size / 32 (integrity verification)

---

## 12. Memory Layout Diagram (Hex Dump Example)

A .btree file containing 3 nodes (96 bytes):

```
Offset  00 01 02 03  04 05 06 07  08 09 0A 0B  0C 0D 0E 0F
------  -----------  -----------  -----------  -----------
0x00    [  minX   ]  [  minY   ]  [  minZ   ]  [  maxX   ]   ← Node 0 (Root, Internal)
0x10    [  maxY   ]  [  maxZ   ]  [right/4  ]  [splitAxis]
                                   = 0x00000010  = 0x00000001
                                   → right at byte 64    axis=Y

0x20    [  minX   ]  [  minY   ]  [  minZ   ]  [  maxX   ]   ← Node 1 (Left child, Leaf)
0x30    [  maxY   ]  [  maxZ   ]  [triOffset]  [cnt|FFFF ]
                                   = 0x00000000  = 0xFFFF0005
                                   → offset=0, count=5

0x40    [  minX   ]  [  minY   ]  [  minZ   ]  [  maxX   ]   ← Node 2 (Right child, Leaf)
0x50    [  maxY   ]  [  maxZ   ]  [triOffset]  [cnt|FFFF ]
                                   = 0x00000005  = 0xFFFF0003
                                   → offset=5, count=3
```

In the example above:
- Root (byte 0): `field6 = 0x10 = 16`, right child at `16 * 4 = 64 = 0x40` ✓
- Left child is at byte 0x20 (immediately after root) ✓
- Right child is at byte 0x40 (as indicated by field6) ✓

Note on field7 byte layout in memory (Little-Endian):
```
Leaf node field7 = count(u16_LE) + 0xFFFF(u16_LE)
  count=5:  05 00 FF FF  → reads as u16[14]=0x0005, u16[15]=0xFFFF
```

---

## 13. FAQ

**Q: Why does field6 need to be multiplied by 4?**  
A: Historical design. The writer stores the offset in 4-byte units (i.e., byteOffset / 4). The reader must restore it to a byte offset. This also allows GPU shaders to use uint32 array indexing directly.

**Q: Which mesh do the triangle indices refer to?**  
A: The same-named `.ply` file. Face order is rearranged during BVH construction, and the `.ply` file is saved with the rearranged order.

**Q: Can triangle index ranges in leaf nodes overlap within a single .btree file?**  
A: No. Each triangle belongs to exactly one leaf node. All leaf `[offset, offset+count)` ranges are non-overlapping and together cover `[0, total_faces)`.

**Q: Can I read all nodes sequentially without traversing the tree structure?**  
A: Yes. You can iterate with `for offset in range(0, file_size, 32)` to read each node. However, the topological (parent-child) relationships require tree traversal to determine.

**Q: Could a splitAxis value of 0xFFFF be misidentified as a leaf?**  
A: No. splitAxis only takes values 0, 1, or 2 and cannot reach 0xFFFF. The leaf marker is located in uint16_high (bytes 30-31). For internal nodes, splitAxis is read as a full uint32 whose upper 16 bits are always 0.

---

## 14. Reference Implementation

Source code locations:
- Writing: `src/Util/MeshUtils.cpp` → `populateBVHBuffer()`, `SaveBVHDataToBin()`
- Data structures: `src/Util/MeshUtils.h` → `BVHNode`, `TriMesh`, `BYTES_PER_NODE`
- Construction: `src/Util/MeshUtils.cpp` → `splitBVHNode()`, `buildBVH()`
