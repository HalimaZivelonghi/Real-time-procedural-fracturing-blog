# Real time procedural fracturing

## Introduction

For this project I worked on creating a real time destruction system applicable to any tipe of mesh, focusing on the optimization aspect a lot, as my main goal was to make this program as flexible and reusable as possible. This is why went for real time fracturing rather than precomputed, this way any mesh can be used easily without needing any precalculations as well making it versatile for generating different fracture geometry. However to achieve this I had to focus heavily on performace, testing the systems's limit with multiple stress tests. I will include a small section on optimization, however this part is very engine specific and was a result of in depth profiling, which might be different based on the starting point that's being used.

## Mesh splitting

The first step to implement fracturing was creating a mesh splitter system capable of dividing a mesh along a plane, creating two different ones. This process is well explained in [this paper](https://www.geometrictools.com/Documentation/ClipMesh.pdf), which was the one I followed for the geometry calculations and I recommend taking a look at it.

### Initialization

Firstly I created structs to store Vertices, Edges and Faces that store all their needed information like position, index, adjacent geometry and side, filling them with the mesh geometry. Then I made ClipMode struct that lets me choose which side of the mesh I want as a result, saving me a lot of heavy unnecessary calculations. 
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

```cpp
int CMesh::ProcessVertices(glm::vec4 plane)
{
    // check side
    glm::vec3 normal = glm::vec3(plane.x, plane.y, plane.z);
    float c = plane.w;
    int positive = 0, negative = 0;
    for (int i = 0; i < m_cVertices.size(); i++)
    {
        m_cVertices[i].distance = glm::dot(normal, m_cVertices[i].point) - c;
        if (m_cVertices[i].distance >= m_testEpsilon)
        {
            m_cVertices[i].side = 1;
            positive++;
        }
        else if (m_cVertices[i].distance <= -m_testEpsilon)
        {
            m_cVertices[i].side = -1;
            negative++;
        }
        else
        {
            m_cVertices[i].side = 0;
            m_cVertices[i].distance = 0;
        }
    }

    if (negative == 0) return 1;
    if (positive == 0) return -1;
    else return 0;
}
```

### Process edges

With every vertex classified, edges can be checked. If both endpoints are on the same side the edge stays intact and gets the same label. However when an edge crosses the plane the intersection point is calculated using linear interpolation between the two endpoints based on their signed distances, and a new vertex is inserted there. The edge is then split: one half goes to the positive side, one to the negative, each referencing the new intersection vertex as their shared endpoint. This is what produces the clean cut line across the mesh.

```cpp
void CMesh::ProcessEdges(ClipMode mode)
{
    // check side
    size_t edgeCount = m_cEdges.size();
    for (int i = 0; i < edgeCount; i++)
    {
        int s0 = m_cVertices[m_cEdges[i].vertices.first].side;
        int s1 = m_cVertices[m_cEdges[i].vertices.second].side;

        if (s0 == 0 && s1 == 0)
        {
            m_cEdges[i].side = 0;
            continue;
        }

        if (s0 <= 0 && s1 <= 0)
        {
            m_cEdges[i].side = -1;
            continue;
        }

        if (s0 >= 0 && s1 >= 0)
        {
            m_cEdges[i].side = 1;
            continue;
        }

        // edge is cut, replace second vertex with the point on the plane
        float d0 = m_cVertices[m_cEdges[i].vertices.first].distance;
        float d1 = m_cVertices[m_cEdges[i].vertices.second].distance;
        float t = d0 / (d0 - d1);

        glm::vec3 intersect =
            (1.f - t) * m_cVertices[m_cEdges[i].vertices.first].point + t * m_cVertices[m_cEdges[i].vertices.second].point;

        size_t newVertexIdx = m_cVertices.size();
        float epsilon = 0.01f;  
        bool foundExisting = false;

        // check ALL vertices that might be on this edge (not just endpoints)
        for (size_t v = 0; v < m_cVertices.size(); v++)
        {
            if (glm::length(intersect - m_cVertices[v].point) < epsilon)
            {
                newVertexIdx = v;
                foundExisting = true;
                break;
            }
        }

        if (!foundExisting)
        {
            m_cVertices.push_back({intersect, 0.0f, 0, 0});
        }

        CEdge originalEdge = m_cEdges[i];

        if (mode == ClipMode::Positive || mode == ClipMode::Both)
        {
            CEdge posEdge = originalEdge; 
            if (s0 > 0)
            {
                posEdge.vertices.second = newVertexIdx;
                posEdge.side = 1;
            }
            else
            {
                posEdge.vertices.first = newVertexIdx;
                posEdge.side = 1;
            }

            m_cEdges[i] = posEdge;
        }

        if (mode == ClipMode::Negative || mode == ClipMode::Both)
        {
            CEdge negEdge = originalEdge;

            if (s0 > 0)
            {
                negEdge.vertices.first = newVertexIdx;
                negEdge.side = -1;
            }
            else
            {
                negEdge.vertices.second = newVertexIdx;
                negEdge.side = -1;
            }

            size_t negEdgeIdx = m_cEdges.size();
            m_cEdges.push_back(negEdge);

            for (size_t faceIdx : originalEdge.face)  
            {
                if (faceIdx == std::numeric_limits<size_t>::max()) continue;
                m_cFaces[faceIdx].edge.push_back(negEdgeIdx);
            }
        }
    }
}
```

### Process faces

Once edges are processed, each face is checked to determine which side it belongs to. A face whose vertices are all positive goes to the positive side, all negative goes to the negative side. Faces that were cut by the plane now have a mix of positive and negative edges, so they need to be closed: the cut creates an open boundary along the plane, and a closing edge is added to seal each open face. These closing edges also form the new flat face that sits exactly on the cutting plane, which is what gives each mesh half its interior cap.

```cpp
if (GetOpenPolyline(tempFace, start, final))
{
    CEdge closeEdge;
    closeEdge.vertices.first = start;
    closeEdge.vertices.second = final;
    closeEdge.face[0] = i;
    closeEdge.side = side;  

    size_t closeEdgeIdx = m_cEdges.size();
    m_cEdges.push_back(closeEdge);

    m_cFaces[i].edge.push_back(closeEdgeIdx);

    CEdge reversedEdge;
    reversedEdge.vertices.first = final;
    reversedEdge.vertices.second = start;
    reversedEdge.side = 0;

    size_t reversedEdgeIdx = m_cEdges.size();
    m_cEdges.push_back(reversedEdge);
    planeFace.edge.push_back(reversedEdgeIdx);
}
```

```cpp
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

### Clean-up, triangulation and normal calculations

Once all the clipping is done, ConvertMesh takes care of turning the internal mesh representation back into something renderable. First it collects only the vertices that belong to the requested side, building a remapping table so that the new index buffer stays compact and has no gaps from discarded vertices.
Then for each face on the correct side, it filters down to only the relevant edges, sorts them into a proper vertex order, and triangulates using a fan from the first vertex, every polygon becomes a set of triangles sharing that first point. Any face that ends up with fewer than three valid vertices after filtering gets skipped entirely to avoid degenerate geometry.
Normals are computed once per face using the cross product of two edges, then accumulated into each vertex that the face touches. After all faces are processed, every accumulated normal gets normalized, giving smooth shading across shared edges.
Vertex remapping handles a specific problem that comes up at the cut boundary: when the plane slices through the mesh, new intersection vertices get created independently for each edge that crosses it, which can leave multiple vertices sitting at nearly identical positions along the cut. The remapping pass finds these duplicates by comparing positions within a small epsilon threshold and merges them down to a single vertex, making sure the closing face on the cut plane is properly connected rather than having tiny invisible cracks between its edges.

![alt text](<Annotation 2025-12-04 092311.png>)

![alt text](<Annotation 2025-12-02 114126-1.png>)


![alt text](<Annotation 2025-12-02 113933.png>)
![alt text](<Annotation 2025-12-02 114019.png>)
![alt text](<Annotation 2025-12-04 092238.png>)


---

## Voronoi generation

With the mesh splitter in place, the next step was figuring out how to generate a realistic fracture pattern. The approach I went with is Voronoi fracturing: a set of random seed points is scattered inside the mesh's bounding volume, and the space is divided so that every point in the mesh belongs to the cell of its nearest seed. Cutting the mesh along all the cell boundaries produces the final fragments. The reason this works well for destruction is that Voronoi cells are always convex, which makes them straightforward to simulate as physics bodies and gives fracture patterns that look naturally irregular.

[VORO++]

The fracturer takes the original mesh, generates a set of random seed points distributed within the mesh's axis-aligned bounding box, and hands them to Voro++. For each cell the library returns, the cell's faces are extracted and each one is used as a plane to progressively clip a copy of the original mesh down to just the geometry that belongs to that cell. The result is one small mesh per cell, which becomes a debris piece.

![alt text](<Annotation 2026-03-18 222407.png>)

## Optimization

The biggest challenge was getting the geometry to be correct and stable. Fragments were coming out degenerate (wrong normals, broken faces, vertices growing exponentially at each cut) and a lot of time went into debugging and fixing those issues to get clean output.
The biggest problem I faced was the vertices growing to such a number that they were exceeding OpenGL's limit. This was mostly because I was using flat normals, which means at each cut vertices get triplicated, but I couldn't use smooth normals since the meshes are very pointy and those things together look very bad. After a lot of trial and error the best solution was to use Jolt's ConvexHull to simplify the geometry, which reduced the average vertex number from around 10.000-100.000 to 15-30.

## Trigger destruction and apply physics