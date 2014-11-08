Notes
=====

I'm forcing myself to write down some of the ideas I've found in various papers pertaining to position-based
dynamics. I have to keep reminding myself I will forget all of this stuff *eventually*, so that this is time
well-spent. Hopefully these notes are at least vaguely comprehensible. Code snippets below use the glm
library, which mimics GLSL in C++.


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
// Masses are inverted (w = 1/m) in a separate loop later.
```


Tetrahedral barycentric interpolation
-------------------------------------

A detailed surface mesh (triangles) is embedded into the coarse simulation mesh. This detailed
mesh is used for rendering, but could also be used for better collision detection.
In order to deform it together with the simulation mesh, we can compute the barycentric
coordinates of each detail vertex with respect to its parent tetrahedron. With barycentric
coordinates bx, by, bz, bw (real numbers, sum equals 1), and parent tetrahedron vertices x0, x1, x2, x3 (positions),
the detail vertex position is just a linear combination

```
x = bx*x0 + by*x1 + bz*x2 + bw*x3
```

This way, all distortions of the tetrahedra in the simulation mesh are inherited by the detail
surface mesh. Another benefit is that we only need to upload the simulation mesh changes to the GPU,
vertex buffer of the detail mesh doesn't change.

To compute barycentric coordinates, the brute force approach is simply solving a system
of 4 linear equations (one for each dimension of the 3D space, and an additional equation
ensuring the barycentric coordinates sum to 1). In matrix form:

```
// tetraIndex = index of the closest surface tetrahedron.
// mTetra is the Tetrahedralization struct
const Tetrahedron& tet = mTetra.surfaceTetrahedra.at(tetraIndex);
const Point4 x0 = Point4(mTetra.vertices.at(tet.i0), 1.f);
const Point4 x1 = Point4(mTetra.vertices.at(tet.i1), 1.f);
const Point4 x2 = Point4(mTetra.vertices.at(tet.i2), 1.f);
const Point4 x3 = Point4(mTetra.vertices.at(tet.i3), 1.f);
const Matrix4 T(x0, x1, x2, x3);
const Point4 baryCoords = inverse(T) * Point4(v.x, v.y, v.z, 1.f);
ASSERT(abs(baryCoords.x + baryCoords.y + baryCoords.z + baryCoords.w - 1.f) < 0.01f);
```

We placed the positions of the tetrahedron vertices into columns of a 4x4 matrix T
(1 in the last row). Writing b = (bx,by,bz,bw) and the initial position of the detail
vertex v = (vx,vy,vz,1), we get

```
T * b = v
b = T^-1 * v
```

Now, barycentric coordinates usually fulfill bi >= 0, but in my case detail vertices
can be outside of their tetrahedra (each vertex is assigned to the tetrahedron with
the closest center). I'm storing coordinates as 16-bit signed integers (normalized to [-1,1]
by the GPU):

```
DetailVertex dv;
const float mult = 32767.f / 5.f; // Expand to cover the range of 16-bit signed int.
dv.bcx = static_cast<i16>(clamp(baryCoords.x, -5.f, 5.f) * mult);
dv.bcy = static_cast<i16>(clamp(baryCoords.y, -5.f, 5.f) * mult);
dv.bcz = static_cast<i16>(clamp(baryCoords.z, -5.f, 5.f) * mult);
dv.bcw = static_cast<i16>(clamp(baryCoords.w, -5.f, 5.f) * mult);
dv.tetraIndHi = static_cast<u8>(tetraIndex / 256);
dv.tetraIndLo = static_cast<u8>(tetraIndex % 256);
ASSERT((static_cast<int>(dv.tetraIndHi)*256 + dv.tetraIndLo) == tetraIndex);
```



Deforming normals
-----------------

Deformations also distort normals. Going through detail surface triangles
and recomputing normals would be inefficient. Luckily, there is no need to do that.
The key fact is that deformations are local per tetrahedron (knowing the deformation
of the parent tetrahedron is enough). Using the same equation as above, we have

```
T * b = v
```
in undeformed state. Under deformations, T is replaced by Q (new positions
of tetrahedron vertices) and v by p (new position of detail vertex):

```
Q * b = p
```

Since barycentric coordinates stay the same, it follows

```
p = Q * T^-1 * v
A = Q * T^-1
```

Matrix A describes the deformation of the tetrahedron. It has the form

```
    |       |
A = |  B   t|
    |       |
    |0 0 0 1|
```

where t is a translation and B contains rotation/scaling.
Translation is irrelevant for normals, we are only interested in B.
I store inverses of T (here called P) in a precomputation step:

```
for (const Tetrahedron& t: mTetra.surfaceTetrahedra) {
    const Point4 x0 = Point4(mTetra.vertices.at(t.i0), 1.f);
    const Point4 x1 = Point4(mTetra.vertices.at(t.i1), 1.f);
    const Point4 x2 = Point4(mTetra.vertices.at(t.i2), 1.f);
    const Point4 x3 = Point4(mTetra.vertices.at(t.i3), 1.f);
    const Matrix4 P(x0, x1, x2, x3);
    mInversesOfP.push_back(inverse(P));
}
```

After the physics simulation step is done (each frame), inverse transposes
of matrices B are computed and relayed to the GPU:

```
for (size_t i = 0; i < numSurfaceTetrahedra; ++i) {
    const Tetrahedron& t = mTetra.surfaceTetrahedra.at(i);
    const Point4 x0 = Point4(mParticlePositions.at(t.i0), 1.f);
    const Point4 x1 = Point4(mParticlePositions.at(t.i1), 1.f);
    const Point4 x2 = Point4(mParticlePositions.at(t.i2), 1.f);
    const Point4 x3 = Point4(mParticlePositions.at(t.i3), 1.f);
    const Matrix4 Q(x0, x1, x2, x3);
    const Matrix4 A = Q * mInversesOfP.at(i);
    mQs.push_back(Q); // Q * b = new position.
    mTetrahedraITT.push_back(inverseTranspose(Matrix3(A)));
}
```

I found this in Interactive Virtual Materials [Muller, Gross, 2004].



Tetrahedral signed volume constraint in PBD
-------------------------------------------

I wrote about PBD constraints I used in this demo in a separate [pdf](http://matejd.github.io/DeformableArmadillo/docs/pbd-vector-calculus-gradients.pdf).



Continuous mechanics in PBD
---------------------------
