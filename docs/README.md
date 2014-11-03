Notes
=====

I'm forcing myself to write down some of the ideas I've found in various papers pertaining to position-based
dynamics. I have to keep reminding myself I will forget all of this stuff *eventually*, so that this is time
well-spent.


Tetrahedron
-----------

Tetrahedra are used all over the place in engineering fields. Complex 3D shapes are commonly broken down
into tetrahedra and this mesh is then used for e.g. finite element analysis. I'm using tetrahedra
to place and connect mass particles (position-based dynamics). Vertices of the tetrahedra correspond to
locations, mass is distributed equally to all 4 vertices/particles:

```
const Point3& X0 = particlePositions[t.i0];
const Point3& X1 = particlePositions[t.i1];
const Point3& X2 = particlePositions[t.i2];
const Point3& X3 = particlePositions[t.i3];

const Matrix3 shape(X0-X3, X1-X3, X2-X3);
const float volume = abs(determinant(shape) / 6.f);
const float quarterMass = volume * 0.25f * density;
particleInvMasses[t.i0] += quarterMass;
particleInvMasses[t.i1] += quarterMass;
particleInvMasses[t.i2] += quarterMass;
particleInvMasses[t.i3] += quarterMass;
```


Tetrahedral barycentric interpolation
-------------------------------------


Deforming normals
-----------------


Tetrahedral signed volume constraint in PBD
-------------------------------------------


Continuous mechanics in PBD
---------------------------
