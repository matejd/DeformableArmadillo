#include "SimulationConstraint.hpp"
#include <limits>

using namespace glm;

void SimulationCon::setSimulationData(const Vector<Point3>& positions,
                                      const Vector<Tetrahedron>& tetrahedra,
                                      const float density)
{
    ASSERT(positions.size() > 0);
    ASSERT(tetrahedra.size() > 0);
    LOG(positions.size());
    LOG(tetrahedra.size());

    const int numParticles = positions.size();
    const int numTetrahedra = tetrahedra.size();

    // Clear in case setSimulationData was already called (resetting the simulation).
    particlePositions.clear();
    particlePredictedPositions.clear();
    particleVelocities.clear();
    particleInvMasses.clear();
    particleFlags.clear();
    particleExternalForces.clear();

    particlePositions = positions;
    particlePredictedPositions = positions;
    particleVelocities.reserve(numParticles);
    particleInvMasses.reserve(numParticles);
    particleFlags.reserve(numParticles);
    particleExternalForces.reserve(numParticles);

    for (int i = 0; i < numParticles; ++i) {
        particleVelocities.push_back(Vec3(0.f));
        particleInvMasses.push_back(0.f);
        particleFlags.push_back(ParticleFlag::Collidable);
        particleExternalForces.push_back(Vec3(0.f));
    }
    ASSERT(particlePositions.size() == particlePredictedPositions.size());
    ASSERT(particlePositions.size() == particleExternalForces.size());

    for (const Tetrahedron& t: tetrahedra) {
        ASSERT(t.i0 < numParticles &&
               t.i1 < numParticles &&
               t.i2 < numParticles &&
               t.i3 < numParticles);
        const Point3& X0 = particlePositions[t.i0];
        const Point3& X1 = particlePositions[t.i1];
        const Point3& X2 = particlePositions[t.i2];
        const Point3& X3 = particlePositions[t.i3];
        ASSERT(X0 != X1);
        ASSERT(X0 != X2);
        ASSERT(X0 != X3);
        ASSERT(X1 != X2);
        ASSERT(X1 != X3);
        ASSERT(X2 != X3);
        // Compute the volume of this tetrahedron. Assign a quater of the
        // element's mass to each vertex.
        const Matrix3 shape(X0-X3, X1-X3, X2-X3);
        const float volume = abs(determinant(shape) / 6.f);
        const float quarterMass = volume * 0.25f * density;
        ASSERT(quarterMass > 0.f);
        particleInvMasses[t.i0] += quarterMass;
        particleInvMasses[t.i1] += quarterMass;
        particleInvMasses[t.i2] += quarterMass;
        particleInvMasses[t.i3] += quarterMass;
    }
#ifdef DEBUG
    float totalMass = 0.f;
    for (int i = 0; i < numParticles; ++i)
        totalMass += particleInvMasses[i];
    LOG(totalMass);
#endif

    for (int i = 0; i < numParticles; ++i) {
        if (particleInvMasses[i] == 0.f)
            continue;
        ASSERT(particleInvMasses[i] != 0.f);
        if (particlePositions[i].y < -0.1f) // Fixate bottom particles.
            particleInvMasses[i] = (1.f / particleInvMasses[i]) * pow(max(0.f, particlePositions[i].y+0.5f), 1.5f);
        else
            particleInvMasses[i] = 1.f / particleInvMasses[i];
    }

    distanceConstraints.clear();
    volumeConstraints.clear();
    for (const Tetrahedron& t: tetrahedra) {
        const Point3& X0 = particlePositions[t.i0];
        const Point3& X1 = particlePositions[t.i1];
        const Point3& X2 = particlePositions[t.i2];
        const Point3& X3 = particlePositions[t.i3];
        const float w0 = particleInvMasses[t.i0];
        const float w1 = particleInvMasses[t.i1];
        const float w2 = particleInvMasses[t.i2];
        const float w3 = particleInvMasses[t.i3];
        // Each tetrahedron has 6 edges:
        // 01, 02, 03,
        // 12, 13,
        // 23
        // where 01 means edge between vertex 0 and vertex 1.
        // DistanceConstraint(p1Idx, p2Idx, p1InvMass, p2InvMass, restSquaredDistance)
        // When both particles are immovable (masses equal to infinity), don't even bother adding a distance constraint.
        if (w0 > 0.f || w1 > 0.f) distanceConstraints.insert(DistanceConstraint(t.i0, t.i1, w0, w1, dot(X1-X0, X1-X0)));
        if (w0 > 0.f || w2 > 0.f) distanceConstraints.insert(DistanceConstraint(t.i0, t.i2, w0, w2, dot(X2-X0, X2-X0)));
        if (w0 > 0.f || w3 > 0.f) distanceConstraints.insert(DistanceConstraint(t.i0, t.i3, w0, w3, dot(X3-X0, X3-X0)));
        if (w1 > 0.f || w2 > 0.f) distanceConstraints.insert(DistanceConstraint(t.i1, t.i2, w1, w2, dot(X2-X1, X2-X1)));
        if (w1 > 0.f || w3 > 0.f) distanceConstraints.insert(DistanceConstraint(t.i1, t.i3, w1, w3, dot(X3-X1, X3-X1)));
        if (w2 > 0.f || w3 > 0.f) distanceConstraints.insert(DistanceConstraint(t.i2, t.i3, w2, w3, dot(X3-X2, X3-X2)));

        // Add a volume constraint to this tetrahedron.
        if (w0+w1+w2+w3 == 0.f)
            continue;
        VolumeConstraint vc;
        vc.i0 = t.i0;
        vc.i1 = t.i1;
        vc.i2 = t.i2;
        vc.i3 = t.i3;
        vc.w0 = w0;
        vc.w1 = w1;
        vc.w2 = w2;
        vc.w3 = w3;
        const Vec3 p1 = X1-X0;
        const Vec3 p2 = X2-X0;
        const Vec3 p3 = X3-X0;
        vc.restVolume = dot(p1, cross(p2, p3)); // This is actually 6 times the volume (and signed)!
        volumeConstraints.push_back(vc);
    }

    LOG(distanceConstraints.size());
    LOG(volumeConstraints.size());

    // Add inversion constraints (not used).
    struct Tet
    {
        int i0, i1, i2, i3;
    };
    Vector<Tet> tets;
    tets.reserve(numTetrahedra*4);
    for (const Tetrahedron& t: tetrahedra) {
        const int i0 = t.i0;
        const int i1 = t.i1;
        const int i2 = t.i2;
        const int i3 = t.i3;
        tets.push_back({i0, i1, i2, i3});
        tets.push_back({i1, i0, i2, i3});
        tets.push_back({i2, i1, i0, i3});
        tets.push_back({i3, i1, i2, i0});
    }

    tetrahedronConstraints.clear();
    for (int i = 0; i < numTetrahedra*4; ++i) {
        ASSERT(i < tets.size());
        const int i0 = tets[i].i0;
        const int i1 = tets[i].i1;
        const int i2 = tets[i].i2;
        const int i3 = tets[i].i3;
        const Point3& X0 = particlePositions[i0];
        const Point3& X1 = particlePositions[i1];
        const Point3& X2 = particlePositions[i2];
        const Point3& X3 = particlePositions[i3];
        const float w0 = particleInvMasses[i0];
        const float w1 = particleInvMasses[i1];
        const float w2 = particleInvMasses[i2];
        const float w3 = particleInvMasses[i3];

        if (w0+w1+w2+w3 == 0.f)
            continue;
        TetrahedronConstraint tc;
        tc.i0 = i0;
        tc.i1 = i1;
        tc.i2 = i2;
        tc.i3 = i3;
        tc.w0 = w0;
        tc.w1 = w1;
        tc.w2 = w2;
        tc.w3 = w3;
        tc.restDistance = dot(normalize(cross(X2-X0, X3-X0)), X1-X0);
        tetrahedronConstraints.push_back(tc);
    }

    LOG(tetrahedronConstraints.size());
}

void SimulationCon::step(float delta)
{
    kCollisionConstraintStiffnessIterInd = 1.f - pow(1.f-kCollisionConstraintStiffness, 1.f/kNumSolverIterations);
    kDistanceConstraintStiffnessIterInd  = 1.f - pow(1.f-kDistanceConstraintStiffness,  1.f/kNumSolverIterations);

    const int numParticles = particlePositions.size();

    for (int i = 0; i < numParticles; ++i) {
        particleVelocities[i] += delta * particleInvMasses[i] * (particleExternalForces[i] + kGravity);
        particlePredictedPositions[i] = particlePositions[i] + delta * particleVelocities[i];
    }

    collisionConstraints.clear();
    for (int i = 0; i < numParticles; ++i) {
        if (!(particleFlags[i] & ParticleFlag::Collidable))
            continue;
        // Ground collision detection only.
        if (particlePredictedPositions[i].y < kGroundHeight) {
            CollisionConstraint cc;
            cc.particleIndex = i;
            cc.location = particlePredictedPositions[i];
            cc.location.y = kGroundHeight;
            cc.normal = Vec3(0.f, 1.f, 0.f);
            collisionConstraints.push_back(cc);
        }
    }

    for (int i = 0; i < kNumSolverIterations; ++i) {
        for (const DistanceConstraint& dc: distanceConstraints)
            projectDistanceConstraint(dc);
        /*for (const TetrahedronConstraint& tc: tetrahedronConstraints)
            projectTetrahedronConstraint(tc);*/
        for (const VolumeConstraint& vc: volumeConstraints)
            projectVolumeConstraint(vc);
        for (const CollisionConstraint& cc: collisionConstraints)
            projectCollisionConstraint(cc);
    }

    const float oneOverDelta = 1.f / delta;
    for (int i = 0; i < numParticles; ++i) {
        particleVelocities[i] = (particlePredictedPositions[i] - particlePositions[i]) * oneOverDelta;
        particlePositions[i] = particlePredictedPositions[i];
    }
/*
    // Restitution and friction. Not needed since we glued the armadillo to the ground!
    for (const CollisionConstraint& cc: collisionConstraints) {
        const Vec3& n = cc.normal;
        ASSERT(cc.particleIndex < particleVelocities.size());
        Vec3& v = particleVelocities[cc.particleIndex];
        v = v - 2.f * kRestitution * dot(v, n) * n; // Reflect velocity in the direction of the collision normal.
        const Vec3 friction = -(v - dot(v, n)*n);
        v += friction;
    }*/
}

void SimulationCon::projectCollisionConstraint(const CollisionConstraint& cc)
{
    ASSERT(cc.particleIndex < particlePredictedPositions.size());
    const Point3& p = particlePredictedPositions[cc.particleIndex];
    const Point3& q = cc.location;
    const Vec3&   n = cc.normal;
    // Project only when invalid (collision is an inequality constraint).
    const float s = dot(p-q, n);
    if (s >= 0.f)
        return;
    const Vec3 delta = -s * kCollisionConstraintStiffnessIterInd * n;
    particlePredictedPositions[cc.particleIndex] += delta;
}

void SimulationCon::projectDistanceConstraint(const DistanceConstraint& dc)
{
    const Point3& p1 = particlePredictedPositions[dc.i1];
    const Point3& p2 = particlePredictedPositions[dc.i2];
    const float w1 = dc.w1;
    const float w2 = dc.w2;
    ASSERT(dc.i1 != dc.i2);
    ASSERT(p1 != p2);
    ASSERT(w1+w2 > 0.f);

    /*
    // Distance (not squared) constraint.
    const float sqrDist = dot(p1-p2, p1-p2);
    if (sqrDist < 0.01f || (w1+w2 < 0.01f))
        return;
    const Vec3 n = (p1-p2) / sqrDist;
    const float s = (sqrDist - dc.restDistance*dc.restDistance) / (w1+w2);
    const Vec3 delta1 = -w1 * s * k * n;
    const Vec3 delta2 =  w2 * s * k * n;*/

    // http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?p=2857#p2857
    // Distance constraint better than Squared distance constraint?
    // In this implementation the usual distance constraint was exploding under high stiffness (k),
    // but squared distance constraint is just fine.
    const Vec3 n = p1-p2;
    const float sqrDist = dot(n, n);
    const float eps = 0.001f;
    if (sqrDist < eps)
        return;
    const float constraintValue = sqrDist - dc.restSquaredDistance;
    const float s = -constraintValue / (2.f*sqrDist*(w1+w2));
    const Vec3 delta1 =  w1 * s * kDistanceConstraintStiffnessIterInd * n;
    const Vec3 delta2 = -w2 * s * kDistanceConstraintStiffnessIterInd * n;

    particlePredictedPositions[dc.i1] += delta1;
    particlePredictedPositions[dc.i2] += delta2;
}

void SimulationCon::projectTetrahedronConstraint(const TetrahedronConstraint& tc)
{
    const Point3& X0 = particlePredictedPositions[tc.i0];
    const Point3& X1 = particlePredictedPositions[tc.i1];
    const Point3& X2 = particlePredictedPositions[tc.i2];
    const Point3& X3 = particlePredictedPositions[tc.i3];
    // Tetrahedron is translated so that X0 is at the origin!
    // Another way of looking at it is that we are working with edges (edge vectors)
    // of the tetrahedron.
    // X1 should be prevented from crossing the plane (X0, X2, X3).
    const Vec3 p1 = X1-X0;
    const Vec3 p2 = X2-X0;
    const Vec3 p3 = X3-X0;

    const Vec3 p2xp3 = cross(p2, p3);
    const float lenp2xp3 = length(p2xp3);
    const float eps = 0.001f;
    if (lenp2xp3 < eps)
        return;
    const Vec3 n = p2xp3/lenp2xp3;
    const float ndotp1 = dot(n, p1);

    // Gradients of the constraint with respect to all 4 nodes.
    const Vec3 q1 = n;
    const Vec3 q2 =  (cross(p3, p1) + cross(n, p3)*ndotp1) / lenp2xp3;
    const Vec3 q3 = -(cross(p2, p1) + cross(n, p2)*ndotp1) / lenp2xp3;
    const Vec3 q0 = -q1-q2-q3;

    const float constraintValue = ndotp1 - tc.restDistance;
    const float w0 = tc.w0;
    const float w1 = tc.w1;
    const float w2 = tc.w2;
    const float w3 = tc.w3;
    const float sumDenom = w0*dot(q0,q0) +
                           w1*dot(q1,q1) +
                           w2*dot(q2,q2) +
                           w3*dot(q3,q3);
    if (sumDenom < eps)
        return;
    const float s = -constraintValue / sumDenom;
    particlePredictedPositions[tc.i0] += w0 * s * kDistanceConstraintStiffnessIterInd * q0;
    particlePredictedPositions[tc.i1] += w1 * s * kDistanceConstraintStiffnessIterInd * q1;
    particlePredictedPositions[tc.i2] += w2 * s * kDistanceConstraintStiffnessIterInd * q2;
    particlePredictedPositions[tc.i3] += w3 * s * kDistanceConstraintStiffnessIterInd * q3;
}

void SimulationCon::projectVolumeConstraint(const VolumeConstraint& vc)
{
    const Point3& X0 = particlePredictedPositions[vc.i0];
    const Point3& X1 = particlePredictedPositions[vc.i1];
    const Point3& X2 = particlePredictedPositions[vc.i2];
    const Point3& X3 = particlePredictedPositions[vc.i3];

    const Vec3 p1 = X1-X0;
    const Vec3 p2 = X2-X0;
    const Vec3 p3 = X3-X0;

    const float constraintValue = dot(p1, cross(p2, p3)) - vc.restVolume;
    const float w0 = vc.w0;
    const float w1 = vc.w1;
    const float w2 = vc.w2;
    const float w3 = vc.w3;

    const Vec3 q1 = cross(p2, p3);
    const Vec3 q2 = cross(p3, p1);
    const Vec3 q3 = cross(p1, p2);
    const Vec3 q0 = -q1-q2-q3;
    const float sumDenom = w0*dot(q0,q0) +
                           w1*dot(q1,q1) +
                           w2*dot(q2,q2) +
                           w3*dot(q3,q3);
    const float eps = 0.001f;
    if (sumDenom < eps)
        return;
    const float s = -constraintValue / sumDenom;
    particlePredictedPositions[vc.i0] += w0 * s * kDistanceConstraintStiffnessIterInd * q0;
    particlePredictedPositions[vc.i1] += w1 * s * kDistanceConstraintStiffnessIterInd * q1;
    particlePredictedPositions[vc.i2] += w2 * s * kDistanceConstraintStiffnessIterInd * q2;
    particlePredictedPositions[vc.i3] += w3 * s * kDistanceConstraintStiffnessIterInd * q3;
}
