#ifndef Tetrahedralization_Hpp
#define Tetrahedralization_Hpp

#include <Platform.hpp> // Vector, Point3, glm.

struct Triangle
{
    int i0, i1, i2;
};

struct Tetrahedron
{
    int i0, i1, i2, i3;

    Tetrahedron() = default;
    Tetrahedron(int i0, int i1, int i2, int i3):
        i0(i0), i1(i1), i2(i2), i3(i3) {}
};

STATIC_ASSERT(sizeof(Triangle) == 12);
STATIC_ASSERT(sizeof(Tetrahedron) == 16);

struct Tetrahedralization
{
    Vector<Point3> vertices;
    Vector<Tetrahedron> tetrahedra; // Indices into vertices[].
    Vector<Tetrahedron> surfaceTetrahedra; // Those that have at least one face on the surface.
    Vector<Triangle> surfaceTriangles;
};

struct TetraParams
{
    float facetAngle = 30;
    float facetSize = 0.2;
    float facetDistance = 0.05;
    float cellRadiusEdgeRatio = 2;
    float cellSize = 0.2;
};

// Uses CGAL library to compute tetrahedralization of a surface mesh. Input consists of
// a list of surface vertices and a list of triangle indices. Tetrahedralization
// is returned as a list of vertices (not the same as input) and a list
// of tetrahedra (each tetrahedron consists of 4 indices into the list of vertices).
// Additionally, a list of "surface tetrahedra" (at least one face on the outer mesh boundary)
// and a list of surface triangles is also computed. TetraParams are well documented at
// http://doc.cgal.org/latest/Mesh_3/index.html#Chapter_3D_Mesh_Generation
#define EXPORT __attribute__ ((visibility ("default")))
EXPORT extern bool tetrahedralize(const Vector<Point3>& vertices,
                                  const Vector<int>& indices,
                                  const TetraParams& params,
                                  Tetrahedralization* tetrahedralization); // In-out.

#endif // Tetrahedralization_Hpp
