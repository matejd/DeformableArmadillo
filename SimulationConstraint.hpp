#ifndef SimulationConstraint_Hpp
#define SimulationConstraint_Hpp

#include "Tetrahedralization.hpp" // struct Tetrahedron (4 indices).

enum ParticleFlag : u8
{
    Collidable = 1,
    Inverted   = 1 << 1
};

/// (Squared) distance constraint.
struct DistanceConstraint
{
    int i1, i2;                // Particle indices.
    float w1, w2;              // Masses (inverted) are immutable in our sim, let's cache them here.
    float restSquaredDistance; // Squared distance instead of just plain distance.

    DistanceConstraint(int i1, int i2, float w1, float w2, float restSquaredDistance):
        i1(i1), i2(i2), w1(w1), w2(w2),
        restSquaredDistance(restSquaredDistance) {}

    // We'll put distance constraints on edges of the tetrahedra. In order
    // not to duplicate constraints (edges are shared between tetrahedra), we'll
    // use std::set to keep just the unique ones.
    friend bool operator < (const DistanceConstraint& dc1, const DistanceConstraint& dc2)
    {
        // Sort lexicographically.
        if (dc1.i1 < dc2.i1)
            return true;
        if (dc1.i1 == dc2.i1 && dc1.i2 < dc2.i2)
            return true;
        return false;
    }
};

/// Constraint designed to prevent element inversion.
/// Each tetrahedron node needs to be prevented from
/// crossing the plane of the other three nodes.
///
/// This particular constraint turns out to be less useful because it's expensive (4 constraints per tetrahedron).
/// We're gonna use VolumeConstraint instead. Left in here for completeness' sake.
struct TetrahedronConstraint
{
    int i0, i1, i2, i3;
    float w0, w1, w2, w3;
    float restDistance; // Signed distance between i0 and plane of the other 3 nodes.
};

/// Designed to keep tetrahedra at their rest volume. Handles negative volume
/// values properly (tries to un-invert).
struct VolumeConstraint
{
    int i0, i1, i2, i3;
    float w0, w1, w2, w3;
    float restVolume; // Signed volume!
};

struct CollisionConstraint
{
    int particleIndex;
    Point3 location;
    Vec3 normal;
};

STATIC_ASSERT(sizeof(DistanceConstraint) == 5*4);
STATIC_ASSERT(sizeof(VolumeConstraint) == (4+4+1)*4);
STATIC_ASSERT(sizeof(TetrahedronConstraint) == (4+4+1)*4);
STATIC_ASSERT(sizeof(CollisionConstraint) == (1+3+3)*4);

/// Position-based simulation (dynamics).
class SimulationCon
{
public:
    void setSimulationData(const Vector<Point3>& particlePositions,
                           const Vector<Tetrahedron>& tetrahedra,
                           const float density);

    // Advance simulation, delta is in seconds.
    void step(float delta);

    const Vector<Point3>& getParticlePositions()
    {
        // In the same order initial positions were passed in (setSimulationData).
        return particlePositions;
    }

    void setParticleExternalForce(const int particleIndex, const Vec3& force)
    {
        ASSERT(particleIndex < particleExternalForces.size());
        particleExternalForces[particleIndex] = force;
    }

private:
    // Arrays of the same length.
    Vector<Point3> particlePositions;
    Vector<Point3> particlePredictedPositions;
    Vector<Point3> particleVelocities;
    Vector<float> particleInvMasses;
    Vector<u8> particleFlags;
    Vector<Vec3> particleExternalForces;

    // Collision constraint are dynamically generated, others are
    // generated on startup only.
    Vector<CollisionConstraint> collisionConstraints;
    Vector<TetrahedronConstraint> tetrahedronConstraints;
    Vector<VolumeConstraint> volumeConstraints;
    Set<DistanceConstraint> distanceConstraints;

    void projectCollisionConstraint(const CollisionConstraint&);
    void projectDistanceConstraint(const DistanceConstraint&);
    void projectTetrahedronConstraint(const TetrahedronConstraint&);
    void projectVolumeConstraint(const VolumeConstraint&);

    friend class DeformableArmadillo;

    // Relevant constants, without const because we want to modify
    // them through AntTweakBar.
    int kNumSolverIterations = 20;
    float kCollisionConstraintStiffness = 0.9f;
    float kDistanceConstraintStiffness = 1.0f;
    float kRestitution = 0.8f;
    Vec3 kGravity = Vec3(0.f, -1.8f, 0.f);
    float kGroundHeight = -0.55f;

    // Constraint stiffness values modified to be independent of the
    // number of iterations used (see Muller's paper).
    // These are updated every time step() is called (AntTweakBar is set up
    // to modify stiffness constants above).
    float kCollisionConstraintStiffnessIterInd = 0.9f;
    float kDistanceConstraintStiffnessIterInd = 1.0f;
};

#endif // Simulation_Hpp
