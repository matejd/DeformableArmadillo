#include <App.hpp>
#include <OrbitalCamera.hpp>
#include <TweakBar.hpp>
#include <Timer.hpp>

#include "Tetrahedralization.hpp"
#include "SimulationConstraint.hpp"

// cereal library is included in the Framework under the external folder.
#include <external/cereal/archives/binary.hpp>
#include <external/cereal/types/vector.hpp>

/// Detail surface vertex. Detail surface is embedded into the simulation
/// mesh. It's used only for rendering. Each detail vertex is deformed by
/// its parent tetrahedron. Deformations are transfered through barycentric
/// coordinates. Normal is also deformed appropriately (see the corresponding
/// precomputation step in setup()).
struct DetailVertex
{
    i16 bcx, bcy, bcz, bcw; // Barycentric tetrahedron coords, normalized to [-5, 5] (vertices can be outside of their tetrahedron).
    u8 tetraIndHi, tetraIndLo, nx, ny; // Tetrahedron index (tetraIndHi*256+tetraIndLo) + encoded normal.
};
STATIC_ASSERT(sizeof(DetailVertex) == 8+4);

class DeformableArmadillo: public App
{
public:
    DeformableArmadillo(int argc, char** argv): App(argc, argv) {}

    virtual bool setup() override;
    virtual bool onEvent(const KeyboardEvent) override;
    virtual bool onEvent(const MouseEvent) override;
    virtual void drawFrame() override;

private:
    void advanceSimulation();
    void computeDeformations();
    void checkPicking(const Matrix4& mvp);
    void renderAuxiliaryStuff(const Matrix4& mvp);

    const String kInputMeshFile  = "assets/armadillo_decimated.obj"; // Input data.
    const String kSerializedFile = "assets/armadillo.serialized";    // Serialized DeformableArmadillo.

    Tetrahedralization mTetra; // Contains vertices+tetrahedra of tetrahedral tessellation.
    Vector<Matrix4> mInversesOfP; // One matrix per surface tetrahedron.
    Vector<Matrix4> mQs;
    Vector<Matrix3> mTetrahedraITT; // Inverse transposes, deforming normals.

    IndexBufferID  mDetailMeshIB;
    VertexBufferID mDetailMeshVB;
    VertexFormatID mDetailMeshVF;
    Vector<DetailVertex> mDetailVertices;
    Vector<u16>          mDetailIndices; // About 50k vertices (and ~100k tris).
    ShaderID mDetailMeshShader;

    FramebufferID mShadowFb;
    TextureID mShadowTex;
    const int kShadowTexSize = 1024;

    IndexBufferID  mPlaneIB;
    VertexBufferID mPlaneVB;
    VertexFormatID mPlaneVF;
    ShaderID       mPlaneShader;

    int mPickedParticle = -1;
    Point3 mDragLine[2];
    bool mShowDebugWireframe = false;
    const float kForceStrength = 500.f;
    int mMouseX, mMouseY;
    bool mDragging = false; // True after the user clicks on the deformable object and is deforming it.

    OrbitalCamera mOrbiCam;
    TweakBar mTweakBar;

    // TODO: wrap in some defines for conditional inclusion!
    CpuTimer frameTimer;
    CpuTimer microTimer;
#ifdef EMSCRIPTEN
    typedef CpuTimer GpuTimer;
    double eCumulativeFrameTime = 0.;
#endif
    GpuTimer defRendTimer;
    SampleStats<float, 50> frameTimes;
    SampleStats<float, 50> simTimes;
    SampleStats<float, 50> defInvTimes;
    SampleStats<float, 50> pickingTimes;
    SampleStats<float, 50> defRendTimes;

    double mAccumulator = 0.;
    const double kFixedDeltaMs = 16.; // Milliseconds.
    SimulationCon mSimulation;
    Vector<Point3> mParticlePositionsCurrent;
    Vector<Point3> mParticlePositionsPrevious;
    Vector<Point3> mParticlePositions; // Interpolated to match current rendering time.

public:
    template<typename Archive> void serialize(Archive& archive)
    {
        archive(mTetra, mInversesOfP, mDetailVertices, mDetailIndices);
    }
};

namespace cereal
{
    template <typename Archive> void serialize(Archive& archive, Point3& p)
    {
        archive(p.x, p.y, p.z);
    }

    template <typename Archive> void serialize(Archive& archive, Point4& p)
    {
        archive(p.x, p.y, p.z, p.w);
    }

    template <typename Archive> void serialize(Archive& archive, Matrix4& m)
    {
        archive(m[0], m[1], m[2], m[3]);
    }

    template <typename Archive> void serialize(Archive& archive, DetailVertex& dv)
    {
        archive(dv.bcx, dv.bcy, dv.bcz, dv.bcw,
                dv.tetraIndHi, dv.tetraIndLo,
                dv.nx, dv.ny);
    }

    template <typename Archive> void serialize(Archive& archive, Triangle& t)
    {
        archive(t.i0, t.i1, t.i2);
    }

    template <typename Archive> void serialize(Archive& archive, Tetrahedron& t)
    {
        archive(t.i0, t.i1, t.i2, t.i3);
    }

    template <typename Archive> void serialize(Archive& archive, Tetrahedralization& t)
    {
        archive(t.vertices, t.tetrahedra, t.surfaceTetrahedra,
                t.surfaceTriangles);
    }
}
