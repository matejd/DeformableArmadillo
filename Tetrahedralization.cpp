#include "Tetrahedralization.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/make_mesh_3.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Polyhedral_mesh_domain_3<Polyhedron, Kernel> MeshDomain;
typedef CGAL::Mesh_triangulation_3<MeshDomain>::type MeshTriangulation;
typedef CGAL::Mesh_complex_3_in_triangulation_3<MeshTriangulation> MeshComplex;
typedef CGAL::Mesh_criteria_3<MeshTriangulation> MeshCriteria;

using namespace CGAL::parameters;
using namespace glm;

/// Builds a CGAL polyhedron out of vertex/index arrays.
template <class HalfedgeDataStructure>
class CGALBuilder : public CGAL::Modifier_base<HalfedgeDataStructure>
{
public:
    CGALBuilder(const Vector<Point3>& vertices, const Vector<int>& indices): vertices(vertices), indices(indices) {}
    void operator()(HalfedgeDataStructure& hds)
    {
        typedef typename HalfedgeDataStructure::Vertex Vertex;
        typedef typename Vertex::Point Point;

        CGAL::Polyhedron_incremental_builder_3<HalfedgeDataStructure> B(hds, true);
        const int numVertices = vertices.size();
        const int numFaces    = indices.size()/3;

        B.begin_surface(numVertices, numFaces);
        for (int i = 0; i < numVertices; ++i) {
            const Point3& v = vertices.at(i);
            B.add_vertex(Point(v.x, v.y, v.z));
        }
        for (int i = 0; i < numFaces; ++i) {
            B.begin_facet();
            B.add_vertex_to_facet(indices.at(i*3+0));
            B.add_vertex_to_facet(indices.at(i*3+1));
            B.add_vertex_to_facet(indices.at(i*3+2));
            B.end_facet();
        }
        B.end_surface();
    }

private:
    const Vector<Point3>& vertices;
    const Vector<int>& indices;
};

Tetrahedralization tetrahedralize(const Vector<Point3>& vertices,
                                  const Vector<int>& indices,
                                  const TetraParams& params)
{
    ASSERT(indices.size() % 3 == 0);

    CGALBuilder<Polyhedron::HalfedgeDS> builder(vertices, indices);
    Polyhedron polyhedron;
    polyhedron.delegate(builder);
    // Tetrahedral tessellation!
    const MeshDomain domain(polyhedron);
    const MeshCriteria criteria(facet_angle            = params.facetAngle,
                                facet_size             = params.facetSize,
                                facet_distance         = params.facetDistance,
                                cell_radius_edge_ratio = params.cellRadiusEdgeRatio,
                                cell_size              = params.cellSize);
    const MeshComplex mesh = CGAL::make_mesh_3<MeshComplex>(domain, criteria);
    const MeshTriangulation& triangulation = mesh.triangulation();

    Tetrahedralization tetr;

    // Go through the tetrahedral tesselation ("triangulation"), extract vertices.
    Map<MeshTriangulation::Vertex_handle, int> vertMap; // Maps CGAL vertex handle to an index into vertex array (built in this loop).
    tetr.vertices.reserve(triangulation.number_of_vertices()); // We know the number of vertices, allocate mem just once.
    int vertexIndex = 0;
    for (auto it = triangulation.finite_vertices_begin();
              it != triangulation.finite_vertices_end();
              ++it) {
        vertMap[it] = vertexIndex++;
        const MeshTriangulation::Point point = it->point();
        tetr.vertices.push_back(
                Point3(CGAL::to_double(point.x()),
                       CGAL::to_double(point.y()),
                       CGAL::to_double(point.z())));
    }
    ASSERT(tetr.vertices.size() == triangulation.number_of_vertices());

    // Extract all tetrahedra.
    tetr.tetrahedra.reserve(mesh.number_of_cells_in_complex());
    for (auto it = mesh.cells_in_complex_begin();
              it != mesh.cells_in_complex_end();
              ++it) {
        tetr.tetrahedra.push_back(Tetrahedron(
            vertMap.at(it->vertex(0)),
            vertMap.at(it->vertex(1)),
            vertMap.at(it->vertex(2)),
            vertMap.at(it->vertex(3))));
    }

    // Find surface tetrahedra (at least one face on the outside).
    for (auto it = mesh.cells_in_complex_begin();
              it != mesh.cells_in_complex_end();
              ++it) {
        if (it->is_facet_on_surface(0) || it->is_facet_on_surface(1) ||
            it->is_facet_on_surface(2) || it->is_facet_on_surface(3)) {
            tetr.surfaceTetrahedra.push_back(Tetrahedron(
                vertMap.at(it->vertex(0)),
                vertMap.at(it->vertex(1)),
                vertMap.at(it->vertex(2)),
                vertMap.at(it->vertex(3))));
        }
    }

    // Extract surface triangles.
    for (auto it = mesh.cells_in_complex_begin();
              it != mesh.cells_in_complex_end();
              ++it) {
        // Four facets, multiple can be on the surface.
        // Facet i on the surface -> vertex with index i (0,1,2 or 3) is opposite of
        // the facet.
        for (int facet = 0; facet < 4; ++facet) {
            if (it->is_facet_on_surface(facet)) {
                Triangle triangle;
                int* ptrToIdx = &triangle.i0;
                for (int i = 0; i < 4; ++i) {
                    if (i != facet) {
                        *ptrToIdx = vertMap.at(it->vertex(i));
                        ptrToIdx++;
                    }
                }
                // Make sure the order of vertices is consistent (triangle
                // normals should all point outside when computed by the cross product (x1-x0) x (x2-x0)).
                const Point3 x0 = tetr.vertices.at(triangle.i0);
                const Point3 x1 = tetr.vertices.at(triangle.i1);
                const Point3 x2 = tetr.vertices.at(triangle.i2);
                const Point3 q  = tetr.vertices.at(vertMap.at(it->vertex(facet)));
                const Vec3 normal = cross(x1-x0, x2-x0);
                const bool normalFlipNeeded = (dot(normal, q-x0) > 0.f);
                if (normalFlipNeeded) {
                    // Swap order.
                    int tmp = triangle.i1;
                    triangle.i1 = triangle.i2;
                    triangle.i2 = tmp;
                }
                tetr.surfaceTriangles.push_back(triangle);
            }
        }
    }

    return tetr;
}
