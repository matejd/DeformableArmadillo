#ifndef Tetrahedralization_Hpp
#define Tetrahedralization_Hpp

#include <Platform.hpp>

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

#define EXPORT __attribute__ ((visibility ("default")))
// CGAL tetrahedralization.
EXPORT extern bool tetrahedralize(const Vector<Point3>& vertices,
                                  const Vector<int>& indices,
                                  const TetraParams& params,
                                  Tetrahedralization* tetrahedralization); // Output.

#endif // Tetrahedralization_Hpp
