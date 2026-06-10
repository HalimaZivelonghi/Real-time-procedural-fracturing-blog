# Real time procedural fracturing

## Introduction

For this project I worked on creating a real time destruction system applicable to any tipe of mesh, focusing on the optimization aspect a lot, as my main goal was to make this program as flexible and reusable as possible. That is why went for real time fracturing rather than precomputed: this way any mesh can be used easily without needing any precalculations, as well making it versatile for generating different fracture geometry. However to achieve this I had to focus heavily on performace, testing the systems's limit with multiple stress tests. I will include a small section on optimization, however this part is very engine specific and was a result of in depth profiling, which might be different based on the starting point that's being used.

## Mesh splitting

The first step to implement fracturing was creating a mesh splitter system capable of dividing a mesh along a plane, creating two different ones. This process is well explained in [this paper](https://www.geometrictools.com/Documentation/ClipMesh.pdf), which was the one I followed for the geometry calculations and I recommend taking a look at it.

![alt text](<Annotation 2025-12-02 113933.png>)

### Initialization

Firstly I created structs to store Vertices, Edges and Faces that store all their needed information like position, index, adjacent geometry and side, filling them with the mesh geometry. Then I made a ClipMode struct that lets me choose which side of the mesh I want as a result, saving me a lot of heavy unnecessary calculations. 
The core clipping logic is as follows, going over vertices, edges and faces in order and offering early outs if the Clipmode is not the desired one.

```cpp
int CMesh::Clip(glm::vec4 plane, ClipMode mode)
{ 
    if (ProcessVertices(plane) == 1) // all vertices on positive side
        return 1;
    else if (ProcessVertices(plane) == -1) // all vertices on negative side
        return -1;

    ProcessEdges(mode);
    ProcessFaces(plane, mode);

    return 0;
}
```

### Process vertices

The function goes over the mesh and calculates the distance of every vertex relative to the cutting plane, classifying them by positive and negative based on the side they are on; this classification drives everything that follows, no geometry is modified yet but its essential for the following calculations.

### Process edges

With every vertex classified, edges can be checked. If both endpoints are on the same side the edge stays intact and gets the same label. However when an edge crosses the plane the intersection point is calculated using linear interpolation between the two endpoints based on their signed distances, and a new vertex is inserted there. The edge is then split: one half goes to the positive side, one to the negative, each referencing the new intersection vertex as their shared endpoint. This is what produces the clean cut line across the mesh.

### Process faces

Once edges are processed, each face is checked to determine which side it belongs to. A face whose vertices are all positive goes to the positive side, all negative goes to the negative side. Faces that were cut by the plane now have a mix of positive and negative edges, so they need to be closed: the cut creates an open boundary along the plane, and a closing edge is added to seal each open face. These closing edges also form the new flat face that sits exactly on the cutting plane, which is what gives each mesh half its interior cap.
As follows is the logic to check if a face is left open and to close it accordingly.

```cpp
// check if the face is left open
bool CMesh::GetOpenPolyline(CFace face, size_t& start, size_t& final)
{ 
    // check for lonely vertices in the face
    for (size_t j : face.edge)
    {
        m_cVertices[m_cEdges[j].vertices.first].occurs = 0;
        m_cVertices[m_cEdges[j].vertices.second].occurs = 0;
    }
    
    for (size_t j : face.edge)
    {
        m_cVertices[m_cEdges[j].vertices.first].occurs++;
        m_cVertices[m_cEdges[j].vertices.second].occurs++;
    }
    
    start = std::numeric_limits<size_t>::max();
    final = std::numeric_limits<size_t>::max();
    
    // finde the verices that need to be connected
    for (size_t j : face.edge)
    {
        size_t i0 = m_cEdges[j].vertices.first;
        size_t i1 = m_cEdges[j].vertices.second;
        
        if (m_cVertices[i0].occurs == 1)
            final = i0;  
        if (m_cVertices[i1].occurs == 1) 
            start = i1;  
    }
    
    return start != std::numeric_limits<size_t>::max();
}
```
```cpp
// if the face is open, create an edge to close it
if (GetOpenPolyline(tempFace, start, final))
{
    Edge closeEdge;
    closeEdge.vertices.first = start;
    closeEdge.vertices.second = final;
    closeEdge.face[0] = i;
    closeEdge.side = side;  

    size_t closeEdgeIdx = Edges.size();
    Edges.push_back(closeEdge);

    Faces[i].edge.push_back(closeEdgeIdx);

    Edge reversedEdge;
    reversedEdge.vertices.first = final;
    reversedEdge.vertices.second = start;
    reversedEdge.side = 0;

    size_t reversedEdgeIdx = Edges.size();
    Edges.push_back(reversedEdge);
    planeFace.edge.push_back(reversedEdgeIdx);
}
```

### Clean-up, triangulation and normal calculations

Now the calculation results need to be converted into a renderable mesh. To do this all vertices on the requested side are collected and put into a remapping table, then each face is triangulated in a fan from the first vertex. Faces with fewer than three valid vertices after filtering are skipped to avoid degenerate geometry.

![alt text](<Annotation 2025-12-04 092311.png>)



Normals are computed per face using the cross product, then accumulated into each shared vertex and normalized to reduce the the total number of normals needed compared to storing one per triangle, this is just an intermediate step as at the very end they will be recalculated since flat normals look much better in this case (see optimization section).

![alt text](<Annotation 2025-12-02 114126-1.png>)

One thing to keep in mind to avoid getting degenerate geometry is that new intersection vertices get created independently per edge, which can leave near duplicate vertices sitting right next to each other along the cut. A remapping pass finds these duplicates within a small epsilon threshold and merges them, making sure the closing face has no invisible cracks.

![alt text](<Annotation 2025-12-02 114019.png>)
![alt text](<Annotation 2025-12-04 092238.png>)

---

## Voronoi generation

With a working mesh splitter, the next question was how to generate a realistic fracture pattern. The approach I went with is Voronoi fracturing: random seed points are scattered inside the mesh's bounding volume, and space is divided so every point belongs to the cell of its nearest seed. Cutting the mesh along all the cell boundaries produces the final fragments. Voronoi cells are always convex, which makes them straightforward to hand off to a physics engine and gives fracture patterns that look naturally irregular.

### Voro++

Rather than implementing Voronoi from scratch, I used Voro++, a C++ library designed specifically for 3D Voronoi computations. It handles all the cell geometry and gives the face normals and vertex positions per cell. The container is sized to the mesh's bounding box, with seed points scattered randomly inside it using the cell count passed in. For each computed cell, I extract the face normals and one vertex per face, transform them from cell local space into world space by adding the seed point position, and build a plane from each face. The result is a list of clipping planes per cell that fully defines that cell's geometry.

```cpp
for (int k = 0; k < cell.number_of_faces(); k++)
{
    int face_size = face_vertices[face_start];
    int vertex_index = face_vertices[face_start + 1] * 3;

    glm::vec3 localVertex(vertices[vertex_index], vertices[vertex_index + 1], vertices[vertex_index + 2]);
    glm::vec3 worldVertex = localVertex + seedPoint; 

    glm::vec3 normal(-face_normals[k * 3], -face_normals[k * 3 + 1], -face_normals[k * 3 + 2]);
    float d = glm::dot(normal, worldVertex);
    glm::vec4 plane(normal.x, normal.y, normal.z, d);
    cell_planes.push_back(plane);

    face_start += face_size + 1;
}
```

## Fracturing

With Voronoi planes ready, the Fracturer applies them to produce the actual debris pieces. For each cell, it takes a fresh copy of the original mesh and progressively clips it against every plane in that cell's plane list, keeping only the positive side each time. What's left after all the clips is the geometry belonging to that Voronoi cell.

```cpp
const auto& cell = cellPlanes[k];
std::vector<glm::vec3> currentPositions = m_mesh->GetPositions();
std::vector<uint16_t> currentIndices = m_mesh->GetIndices();

for (size_t j = 0; j < cell.size(); j++)
{
    const auto& plane = cell[j];
    CMesh cmesh;
    cmesh.FillDataCMesh(currentPositions, currentIndices);
    cmesh.Clip(plane, ClipMode::Positive);
    cmesh.GetPositiveData(currentPositions, currentIndices);
}
```

Once all cells are processed, each resulting piece is handed off to Jolt as a convex hull collision shape and added to the scene as an independent physics body.

![alt text](<Annotation 2026-03-18 222407.png>)

<video controls src="BEE 2026-05-21 09-35-40_Trim-1.mp4" title="Title"></video>

## Optimization

Optimization was a core part of this project since real time fracturing is very expensive, every clip operation touches every vertex and edge in the mesh, and with Voronoi fracturing you're doing dozens of cuts per cell.

The first big improvement was reducing vertex count on the final pieces. Raw fracture output can have a lot of geometry, especially on the cut faces, and feeding all of that directly to the physics engine is wasteful. Instead of using the rendered mesh as the collision shape, I run each piece through Jolt's `ConvexHullShapeSettings` after fracturing, which builds a simplified convex hull from the vertices. This massively reduces the polygon count used for physics without visibly affecting the result, since the debris pieces are already roughly convex by nature of being Voronoi cells. This is where the normals are recalculated into flat ones, making the shadng look better without having an insane number of normals.

The second was multithreading the fracture generation. Each Voronoi cell is completely independent, it works on its own copy of the mesh data and writes to its own slot in the output array, which means there are no shared writes and no locking needed. The one constraint is that anything touching OpenGL has to stay on the main thread, so the actual mesh and physics body creation is deferred to after all the fracturing work is done. This split between worker threads for the heavy geometry work and the main thread for GPU work is what makes the threading safe without any synchronization problems.

Thanks to these optimizations, plus some engine specific profiling, I was able to speed up my system of roughly 78%, which made it stable enough to hold multiple fractures at runtime, effectively achieving my initial goal.

<video controls src="BEE 2026-05-06 08-06-36_Trim (2).mp4" title="Title"></video>

![alt text](<Annotation 2026-06-09 130251.png>)