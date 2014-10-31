#include "DeformableArmadillo.hpp"

#include <external/glm/gtc/type_ptr.hpp>
#include <external/glm/gtc/matrix_inverse.hpp>
#include <external/glm/gtx/transform.hpp>

#include <fstream>
#ifdef DEBUG
#include <fenv.h> // Floating point exceptions.
#endif

using namespace glm;

//#define MESH_GEN_PHASE
#ifdef MESH_GEN_PHASE
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "Tetrahedralization.hpp"
#include "Forsyth.hpp"

#endif // MESH_GEN_PHASE

int main(int argc, char** argv)
{
    DeformableArmadillo app(argc, argv);
    return app.exec();
}

bool DeformableArmadillo::setup()
{
#ifdef DEBUG
    // Catch NaNs and Infs when they are computed.
    feenableexcept(FE_INVALID | FE_OVERFLOW);
#endif

#ifdef MESH_GEN_PHASE
    // Import polygonal mesh using Assimp.
    Assimp::Importer assImport;
    const aiScene* assScene = assImport.ReadFile(kInputMeshFile.c_str(),
                                                 aiProcess_GenSmoothNormals      |
                                                 aiProcess_Triangulate           |
                                                 aiProcess_JoinIdenticalVertices /*|
                                                 aiProcess_ImproveCacheLocality*/); // Uses tipsify algorithm (see below).
    ASSERT(assScene);
    ASSERT(assScene->mNumMeshes == 1);
    ASSERT(assScene->mMeshes[0]->HasNormals());
    const aiMesh* assMesh = assScene->mMeshes[0];
    LOG(assScene->mMeshes[0]->mNumVertices);
    LOG(assScene->mMeshes[0]->mNumFaces);

    Vector<Point3> assVertices;
    Vector<Vec3> assNormals;
    Vector<int> assIndices;
    assVertices.reserve(assMesh->mNumVertices);
    assNormals.reserve(assMesh->mNumVertices);
    assIndices.reserve(assMesh->mNumFaces*3);
    mDetailIndices.reserve(assMesh->mNumFaces*3);
    Vector<int> preIndices;
    preIndices.reserve(assMesh->mNumFaces*3);
    for (unsigned int i = 0; i < assMesh->mNumFaces; ++i) {
        const aiFace& f = assMesh->mFaces[i];
        preIndices.push_back(f.mIndices[0]);
        preIndices.push_back(f.mIndices[1]);
        preIndices.push_back(f.mIndices[2]);
    }
    ASSERT(preIndices.size() == assMesh->mNumFaces*3);

    // Tom Forsyth's Linear-Speed Vertex Cache Optimisation algorithm implementation by Martin Storsjo
    // https://github.com/vivkin/forsyth
    Vector<int> postIndices(assMesh->mNumFaces*3, -1);
    forsythReorderIndices(postIndices.data(), preIndices.data(), assMesh->mNumFaces, assMesh->mNumVertices);

    // Reorder vertices in the vertex buffer, but keep the index buffer *order* the same.
    // Index buffer order is already optimized for post-transform vertex cache.
    // This improves *pre*-transform vertex cache. See Forsyth:
    // http://home.comcast.net/~tom_forsyth/papers/fast_vert_cache_opt.html
    Vector<int> remapping(assMesh->mNumVertices, -1); // Index buffer remapping, the order is kept the same!
    ASSERT(remapping.size() == assMesh->mNumVertices);

    for (size_t i = 0; i < postIndices.size(); ++i) {
        ASSERT(postIndices.at(i) != -1);
        if (remapping.at(postIndices.at(i)) == -1) { // Vertex not yet added to the new vertex buffer.
            const int newIndex = assVertices.size(); // Index must be mapped to newIndex.
            remapping.at(postIndices.at(i)) = newIndex;
            const aiVector3D& v = assMesh->mVertices[postIndices.at(i)];
            const aiVector3D& n = assMesh->mNormals[postIndices.at(i)];
            assVertices.push_back(Point3(v.x, v.y, v.z));
            assNormals.push_back(Vec3(n.x, n.y, n.z));
        }
    }
    ASSERT(assVertices.size() == assMesh->mNumVertices);
    ASSERT(assVertices.size() < 65536); // Limits to u16 index type.

    for (size_t i = 0; i < postIndices.size(); ++i) {
        int newIndex = remapping.at(postIndices.at(i));
        ASSERT(newIndex != -1);
        ASSERT(newIndex < 65536);
        assIndices.push_back(newIndex);
        mDetailIndices.push_back(static_cast<u16>(newIndex));
    }

    /* If you want to tetrahedralize mesh with CGAL, uncomment lines below. cereal
     * is used to store the results in a file.
    TetraParams tetraParams;
    const bool success = tetrahedralize(assVertices, assIndices, tetraParams, &mTetra);
    ASSERT(success);*/

    {
        /*std::ofstream outStream("assets/tetr.tmp", std::ios::binary);
        cereal::BinaryOutputArchive oarchive(outStream);
        oarchive(mTetra);
        outStream.close();*/
        std::ifstream inStream("assets/tetr.tmp", std::ios::binary);
        cereal::BinaryInputArchive ia(inStream);
        ia(mTetra);
        inStream.close();
    }

    // Precompute inverses of matrices P (see Interactive Virtual Materials [2004]).
    // These are used to compute rotation/stretch of each tetrahedron (for normals).
    mInversesOfP.reserve(mTetra.surfaceTetrahedra.size());
    for (const Tetrahedron& t: mTetra.surfaceTetrahedra) {
        const Point4 x0 = Point4(mTetra.vertices.at(t.i0), 1.f);
        const Point4 x1 = Point4(mTetra.vertices.at(t.i1), 1.f);
        const Point4 x2 = Point4(mTetra.vertices.at(t.i2), 1.f);
        const Point4 x3 = Point4(mTetra.vertices.at(t.i3), 1.f);
        const Matrix4 P(x0, x1, x2, x3);
        mInversesOfP.push_back(inverse(P));
    }

    // Next precomputation step: assign each detail vertex to its closest surface tetrahedron
    // (the detail vertex might be slightly outside of the tetrahedron). Compute tetrahedral barycentric
    // coordinates of each detail vertex.
    Vector<Point3> tetrahedraCentroids;
    tetrahedraCentroids.reserve(mTetra.surfaceTetrahedra.size());
    for (const Tetrahedron& t: mTetra.surfaceTetrahedra) {
        const Point3 centroid = 0.25f * (mTetra.vertices.at(t.i0) + mTetra.vertices.at(t.i1) +
                                         mTetra.vertices.at(t.i2) + mTetra.vertices.at(t.i3));
        tetrahedraCentroids.push_back(centroid);
    }

    mDetailVertices.reserve(assVertices.size());
    for (size_t i = 0; i < assVertices.size(); ++i) {
        const Point3& v = assVertices.at(i);
        const Vec3& n   = assNormals.at(i);
        float minDistSqr = std::numeric_limits<float>::infinity();
        size_t tetraIndex = 0;
        for (size_t t = 0; t < tetrahedraCentroids.size(); ++t) {
            const Point3& centroid = tetrahedraCentroids.at(t);
            const Vec3 diff = Point3(v.x, v.y, v.z) - centroid;
            const float distSqr = dot(diff, diff);
            if (distSqr < minDistSqr) {
                tetraIndex = t; // Index into surface tetrahedra.
                minDistSqr = distSqr;
            }
        }
        const Tetrahedron& tet = mTetra.surfaceTetrahedra.at(tetraIndex);
        const Point4 x0 = Point4(mTetra.vertices.at(tet.i0), 1.f);
        const Point4 x1 = Point4(mTetra.vertices.at(tet.i1), 1.f);
        const Point4 x2 = Point4(mTetra.vertices.at(tet.i2), 1.f);
        const Point4 x3 = Point4(mTetra.vertices.at(tet.i3), 1.f);
        const Matrix4 T(x0, x1, x2, x3);
        const Point4 baryCoords = inverse(T) * Point4(v.x, v.y, v.z, 1.f);
        ASSERT(abs(baryCoords.x + baryCoords.y + baryCoords.z + baryCoords.w - 1.f) < 0.01f);
        DetailVertex dv;
        //ASSERT(baryCoords.x >= -5.f && baryCoords.x <= 5.f);
        //ASSERT(baryCoords.y >= -5.f && baryCoords.y <= 5.f);
        //ASSERT(baryCoords.z >= -5.f && baryCoords.z <= 5.f);
        //ASSERT(baryCoords.w >= -5.f && baryCoords.w <= 5.f);
        const float mult = 32767.f / 5.f; // Expand to cover the range of 16-bit signed int.
        dv.bcx = static_cast<i16>(clamp(baryCoords.x, -5.f, 5.f) * mult);
        dv.bcy = static_cast<i16>(clamp(baryCoords.y, -5.f, 5.f) * mult);
        dv.bcz = static_cast<i16>(clamp(baryCoords.z, -5.f, 5.f) * mult);
        dv.bcw = static_cast<i16>(clamp(baryCoords.w, -5.f, 5.f) * mult);
        dv.tetraIndHi = static_cast<u8>(tetraIndex / 256);
        dv.tetraIndLo = static_cast<u8>(tetraIndex % 256);
        ASSERT((static_cast<int>(dv.tetraIndHi)*256 + dv.tetraIndLo) == tetraIndex);
        ASSERT(n.x >= -1.f && n.x <= 1.f);
        ASSERT(n.y >= -1.f && n.y <= 1.f);
        ASSERT(n.z >= -1.f && n.z <= 1.f);
        ASSERT(abs(sqrt(n.x*n.x + n.y*n.y + n.z*n.z) - 1.f) < 0.001f);
        // http://aras-p.info/texts/CompactNormalStorage.html
        // Lambert Azimuthal Equal-Area projection.
        // There are some artifacts with this encoding (it's meant to be used with VS normals, not world-space),
        // but negating n.z hides them on the back of the armadillo (shadowed).
        const float f = sqrt(-8.f*n.z + 8.f);
        dv.nx = static_cast<u8>(clamp(n.x/f + 0.5f, 0.f, 1.f)*255.f);
        dv.ny = static_cast<u8>(clamp(n.y/f + 0.5f, 0.f, 1.f)*255.f);
        mDetailVertices.push_back(dv);
    }

    LOG(mTetra.vertices.size());
    LOG(mTetra.tetrahedra.size());
    LOG(mTetra.surfaceTetrahedra.size());
    LOG(mTetra.surfaceTriangles.size());
    LOG(mDetailVertices.size());
    LOG(mDetailIndices.size());

    // Tetrahedral tessellation is a slooow process, we're storing the results.
    std::ofstream outStream(kSerializedFile.c_str(), std::ios::binary);
    cereal::BinaryOutputArchive oarchive(outStream);
    oarchive(*this);
    outStream.close();
#endif // MESH_GEN_PHASE

    mOrbiCam.canvasWidth = canvasWidth;
    mOrbiCam.canvasHeight = canvasHeight;
    mOrbiCam.cameraR = 1.5f;
    mOrbiCam.cameraMinR = 1.2f;
    mOrbiCam.cameraPhi = -2.86f;
    mOrbiCam.cameraTheta = 1.397f;
    mOrbiCam.cameraMaxTheta = PI2 + 0.3f;
    mOrbiCam.cameraTarget.y = 0.f;
    mTweakBar.canvasWidth = canvasWidth;
    mTweakBar.canvasHeight = canvasHeight;
    renderer.setViewport(0, 0, canvasWidth, canvasHeight);

    App::addEventListener(&mTweakBar);
    App::addEventListener(this);
    App::addEventListener(&mOrbiCam);

    // De-serialize tetrahedral tessellation.
    std::ifstream inStream(kSerializedFile.c_str(), std::ios::binary);
    cereal::BinaryInputArchive ia(inStream);
    ia(*this);
    inStream.close();

    // Data the simulation needs (locations of mass particles, their connectivity
    // and the density of the body).
    mSimulation.setSimulationData(mTetra.vertices,
                                  mTetra.tetrahedra,
                                  2.f);

    mTweakBar.addFloat("Collision Stiffness", &mSimulation.kCollisionConstraintStiffness, 0.1f, 1.f, 0.01f);
    mTweakBar.addFloat("Distance Stiffness", &mSimulation.kDistanceConstraintStiffness, 0.05f, 1.f, 0.01f);
    mTweakBar.addInt("Solver Iterations", &mSimulation.kNumSolverIterations, 1, 30, 1);
    mTweakBar.addBool("Debug Wireframe", &mShowDebugWireframe);

    mTweakBar.addFloat("Avg Frame Time", &frameTimes.average);
    mTweakBar.addFloat("Min Frame Time", &frameTimes.min);
    mTweakBar.addFloat("Max Frame Time", &frameTimes.max);
    mTweakBar.addFloat("Avg Def Inv Time", &defInvTimes.average);
    mTweakBar.addFloat("Min Def Inv Time", &defInvTimes.min);
    mTweakBar.addFloat("Max Def Inv Time", &defInvTimes.max);
    mTweakBar.addFloat("Avg Picking Time", &pickingTimes.average);
    mTweakBar.addFloat("Min Picking Time", &pickingTimes.min);
    mTweakBar.addFloat("Max Picking Time", &pickingTimes.max);
    mTweakBar.addFloat("Avg Def Rendering Time", &defRendTimes.average);
    mTweakBar.addFloat("Min Def Rendering Time", &defRendTimes.min);
    mTweakBar.addFloat("Max Def Rendering Time", &defRendTimes.max);
    mTweakBar.addFloat("Avg Sim Time", &simTimes.average);
    mTweakBar.addFloat("Min Sim Time", &simTimes.min);
    mTweakBar.addFloat("Max Sim Time", &simTimes.max);

    mDetailMeshIB = renderer.addIndexBuffer(mDetailIndices);
    mDetailMeshVB = renderer.addVertexBuffer(mDetailVertices);
    mDetailMeshVF = renderer.addVertexFormat({{4, VertexAttribType::Int16, true,  sizeof(DetailVertex), 0},
                                              {4, VertexAttribType::Uint8, false, sizeof(DetailVertex), 8}});
    mDetailMeshShader = renderer.addShader({"assets/deformable.vs"}, {"assets/deformable.fs"});

    struct PlaneVertex
    {
        Point2 position; // x,z, y fixed
        float u,v;
    };
    STATIC_ASSERT(sizeof(PlaneVertex) == 4*4);
    const Vector<PlaneVertex> planeVertices{
        {Point2(-6.f, -6.f), 0.f, 0.f},
        {Point2( 6.f, -6.f), 1.f, 0.f},
        {Point2( 6.f,  6.f), 1.f, 1.f},
        {Point2(-6.f,  6.f), 0.f, 1.f}
    };
    const Vector<u16> planeIndices{0,2,1, 0,3,2};
    mPlaneVB = renderer.addVertexBuffer(planeVertices);
    mPlaneVF = renderer.addVertexFormat({{2, VertexAttribType::Float, false, sizeof(PlaneVertex), 0},
                                        {2, VertexAttribType::Float, false, sizeof(PlaneVertex), 2*4}});
    mPlaneIB = renderer.addIndexBuffer(planeIndices);
    mPlaneShader = renderer.addShader({"assets/plane.vs"}, {"assets/plane.fs"});

    mShadowFb = renderer.addFramebuffer();
#ifdef EMSCRIPTEN
    // Single-channel texture is not renderable in WebGL.
    mShadowTex = renderer.addEmptyTexture(kShadowTexSize, kShadowTexSize, PixelFormat::Rgb, PixelType::Ubyte, TextureFilter::Nearest);
#else
    mShadowTex = renderer.addEmptyTexture(kShadowTexSize, kShadowTexSize, PixelFormat::R, PixelType::Ubyte, TextureFilter::Nearest);
#endif
    const RenderbufferID depthBuff = renderer.addRenderbuffer(kShadowTexSize, kShadowTexSize, PixelFormat::Depth16);
    renderer.attachTextureToFramebuffer(mShadowFb, mShadowTex)
            .attachRenderbufferToFramebuffer(mShadowFb, depthBuff)
            .setDefaultFramebuffer();
    return true;
}

bool DeformableArmadillo::onEvent(const KeyboardEvent event)
{
    if (event.action == KeyAction::Press && event.key == Key::Space) {
        mShowDebugWireframe = !mShowDebugWireframe;
        return true;
    }

    return false;
}

bool DeformableArmadillo::onEvent(const MouseEvent event)
{
    if (event.action == MouseAction::Move) {
        mMouseX = event.mouseX;
        mMouseY = event.mouseY;
        return false;
    }

    if (event.action == MouseAction::Press && event.button == MouseButton::Left && mPickedParticle != -1) {
        mDragging = true;
        return true;
    }

    if (mDragging && event.action == MouseAction::Release) {
        if (mPickedParticle != -1)
            mSimulation.setParticleExternalForce(mPickedParticle, Vec3(0.f));
        mDragging = false;
        return true;
    }

    return false;
}

void DeformableArmadillo::checkPicking(const Matrix4& mvp)
{
    microTimer.start();
    if (mDragging) {
        const Point3& p = mParticlePositions.at(mPickedParticle);
        const Point4 projected = mvp * Point4(p, 1.f);
        const float winZ = (projected.z / projected.w) * 0.5f + 0.5f;
        const Vec4 viewport(0.f, 0.f, canvasWidth, canvasHeight);
        const Point3 destWorld = unProject(Point3(mMouseX, canvasHeight-mMouseY, winZ), mvp, mat4(1.f), viewport);
        // Compute a force to apply to this particle.
        const Vec3 diff = destWorld-p;
        const Vec3 force = diff * kForceStrength;
        mSimulation.setParticleExternalForce(mPickedParticle, force);
        mDragLine[0] = p;
        mDragLine[1] = destWorld;
        return;
    }

    mPickedParticle = -1;

    // Go through surface triangles (of the simulation mesh),
    // check whether user's cursor hovers over them.
    const Vec4 viewport(0.f, 0.f, canvasWidth, canvasHeight);
    const Point3 rayOrigin      = unProject(Vec3(mMouseX, canvasHeight-mMouseY, 0.f),  mvp, mat4(1.f), viewport);
    const Vec3 rayDir = normalize(unProject(Vec3(mMouseX, canvasHeight-mMouseY, 1.0f), mvp, mat4(1.f), viewport)
                                  - rayOrigin);

    for (const Triangle& tri: mTetra.surfaceTriangles) {
        const Point3 p[3]{
            mParticlePositions.at(tri.i0),
            mParticlePositions.at(tri.i1),
            mParticlePositions.at(tri.i2)};
        const Vec3 e1 = p[1]-p[0];
        const Vec3 e2 = p[2]-p[0];
        const Vec3 normal = normalize(cross(e1, e2));

        // Ray/triangle intersection test [Real-Time Rendering].
        if (dot(rayDir, normal) > 0.f) // Backface culling.
            continue;
        const Vec3 q = cross(rayDir, e2);
        const float a = dot(q, e1);
        if (abs(a) < 0.0001f)
            continue;
        const float f = 1.f / a;
        const Vec3 s = rayOrigin-p[0];
        const float u = f * dot(s, q);
        if (u < 0.f)
            continue;
        const Vec3 r = cross(s, e1);
        const float v = f * dot(rayDir, r);
        if (v < 0.f || (u+v > 1.f))
            continue;
        // We found an intersection!
        const float t = f * dot(e2, q);
        const Point3 rayEnd = rayOrigin + t*rayDir;
        const float d0 = dot(p[0]-rayEnd, p[0]-rayEnd);
        const float d1 = dot(p[1]-rayEnd, p[1]-rayEnd);
        const float d2 = dot(p[2]-rayEnd, p[2]-rayEnd);
        mPickedParticle = tri.i0;
        if (d1 < d0) mPickedParticle = tri.i1;
        if (d2 < d1 && d2 < d0) mPickedParticle = tri.i2;
    }

    pickingTimes.add(microTimer.elapsed());
}

void DeformableArmadillo::advanceSimulation()
{
    double deltaMs = frameTimer.restart();
    frameTimes.add(deltaMs);
    deltaMs = min(deltaMs, 25.); // Prevent the spiral of death!

    microTimer.start();
    // mAccumulator stores the amout of milliseconds we still
    // need to simulate. The simulation is advanced in fixed-time steps.
    // We interpolate particle positions based on how much
    // time is left in mAccumulator! For more, see e.g.
    // http://gafferongames.com/game-physics/fix-your-timestep/
    mAccumulator += deltaMs;
    while (mAccumulator >= kFixedDeltaMs) {
        mParticlePositionsPrevious = mParticlePositionsCurrent;
        mSimulation.step(kFixedDeltaMs * 0.001 * 0.05); // To seconds and then an additional factor because it behaves nicer!
        mAccumulator -= kFixedDeltaMs;
        mParticlePositionsCurrent = mSimulation.getParticlePositions();
    }

    if (mParticlePositionsPrevious.size() == 0) // Might be true in the first frame.
        mParticlePositionsPrevious = mSimulation.getParticlePositions();

    ASSERT(mParticlePositionsCurrent.size() > 0);
    ASSERT(mParticlePositionsCurrent.size() == mParticlePositionsPrevious.size());

    // Interpolate state.
    const float alpha = mAccumulator / kFixedDeltaMs;
    const int numParticles = mSimulation.getParticlePositions().size();
    mParticlePositions.clear();
    for (int i = 0; i < numParticles; ++i) {
        mParticlePositions.push_back(mParticlePositionsCurrent[i] * alpha +
                                     mParticlePositionsPrevious[i] * (1.f - alpha));
    }

    simTimes.add(microTimer.elapsed());
    // Vector of mParticlePositions has the same size and order as mTetra.vertices.
    // We can therefore use the same indices.
}

void DeformableArmadillo::computeDeformations()
{
    microTimer.start();
    // Compute deformation of each surface tetrahedron. Inverse transposes
    // are used to update detail mesh normals.
    mTetrahedraITT.clear();
    mQs.clear();
    const size_t numSurfaceTetrahedra = mTetra.surfaceTetrahedra.size();
    for (size_t i = 0; i < numSurfaceTetrahedra; ++i) {
        const Tetrahedron& t = mTetra.surfaceTetrahedra.at(i);
        const Point4 x0 = Point4(mParticlePositions.at(t.i0), 1.f);
        const Point4 x1 = Point4(mParticlePositions.at(t.i1), 1.f);
        const Point4 x2 = Point4(mParticlePositions.at(t.i2), 1.f);
        const Point4 x3 = Point4(mParticlePositions.at(t.i3), 1.f);
        const Matrix4 Q(x0, x1, x2, x3);
        const Matrix4 A = Q * mInversesOfP.at(i);
        ASSERT(abs(A[3][3] - 1.f) < 0.01f);
        mQs.push_back(Q);
        mTetrahedraITT.push_back(inverseTranspose(Matrix3(A)));
    }
    ASSERT(mTetrahedraITT.size() < 400);
    ASSERT(mTetrahedraITT.size() == mQs.size());
    ASSERT(mQs.size() == mInversesOfP.size());
    defInvTimes.add(microTimer.elapsed());
}

void DeformableArmadillo::drawFrame()
{
    advanceSimulation();
    computeDeformations();

    const Matrix4 mvp = mOrbiCam.getTransformMatrix();
    const Point3 camPosition = mOrbiCam.getPosition();
    const Point3 lightPosition(-2.f, 2.f, -2.f);
    const Vec3 lightDirection = normalize(Point3(0.f, mSimulation.kGroundHeight, 0.f) - lightPosition);
    const float lightInnerCosAngle = cos(radians(23.f));
    const float lightOuterCosAngle = cos(radians(25.f));
    const Matrix4 lightMvp = perspective(radians(40.f), 1.f, 2.f, 9.f) *
                             lookAt(lightPosition, mOrbiCam.cameraTarget, mOrbiCam.worldUp); // View frustum just big enough to contain the deformable.

    checkPicking(mvp);

    // We've computed everything, let's draw.
    //
    renderer.setFramebuffer(mShadowFb)
            .setViewport(0, 0, kShadowTexSize, kShadowTexSize)
            .setCullMode(CullMode::Front)
            .clear(Rgba(1.f, 0.f, 0.f, 0.f), 1.f);

    // Render only the armadillo (making shadows).
    renderer.setShader(mDetailMeshShader)
            .setUniform1i("shadowGen", 1)
            .setUniform4x4fv("lightMvp", 1, glm::value_ptr(lightMvp))
            .setUniform4x4fv("Qs", mQs.size(), reinterpret_cast<const float*>(&mQs[0][0][0]))
            .setInputAssembler(mDetailMeshVB, mDetailMeshVF, mDetailMeshIB)
            .drawIndexedPrimitives(Primitive::Triangle);

    // Shadow rendered, back to default framebuffer.
    renderer.setDefaultFramebuffer()
            .setViewport(0, 0, canvasWidth, canvasHeight)
            .setCullMode(CullMode::Back)
            .clear(Rgba(0.f, 0.f, 0.f, 0.f), 1.f);

    defRendTimer.start(); // Measure GPU time.
    renderer.setUniform1i("shadowGen", 0)
            .setUniform4x4fv("mvp", 1, glm::value_ptr(mvp))
            .setUniform3fv("lightPosition", 1, glm::value_ptr(lightPosition))
            .setUniform1i("shadowSampler", 0)
            .setTexture(0, mShadowTex)
            .setUniform3fv("camPosition", 1, glm::value_ptr(camPosition))
            .setUniform3x3fv("tetrahedraITT", mTetrahedraITT.size(), reinterpret_cast<const float*>(&mTetrahedraITT[0][0][0]))
            .drawIndexedPrimitives(Primitive::Triangle);
    defRendTimes.add(defRendTimer.elapsed());

    // Draw plane.
    renderer.setShader(mPlaneShader)
            .setUniform4x4fv("mvp", 1, glm::value_ptr(mvp))
            .setUniform4x4fv("lightMvp", 1, glm::value_ptr(lightMvp))
            .setUniform1i("shadowSampler", 0)
            .setTexture(0, mShadowTex)
            .setUniform3fv("lightPosition", 1, glm::value_ptr(lightPosition))
            .setUniform3fv("lightDirection", 1, glm::value_ptr(lightDirection))
            .setUniform1f("lightInnerCosAngle", lightInnerCosAngle)
            .setUniform1f("lightOuterCosAngle", lightOuterCosAngle)
            .setUniform1f("groundY", mSimulation.kGroundHeight)
            .setInputAssembler(mPlaneVB, mPlaneVF, mPlaneIB)
            .drawIndexedPrimitives(Primitive::Triangle);

    renderAuxiliaryStuff(mvp);
    renderer.checkGLError();
}

void DeformableArmadillo::renderAuxiliaryStuff(const Matrix4& mvp)
{
    if (mDragging) {
        const Vector<Point3> line{mDragLine[0], mDragLine[1]};
        renderer.drawLines(mvp, line, Rgba(1.f, 0.f, 0.f, 0.f), 0.01f);
    }

    if (mDragging || (!mDragging && mPickedParticle != -1)) {
        const Point3& p = mParticlePositions.at(mPickedParticle);
        const float scale = 0.1f;
        const Vector<Point3> markerLines{
            p+Vec3(-1.f, 0.f, 0.f)*scale, p+Vec3(1.f, 0.f, 0.f)*scale,
            p+Vec3( 0.f,-1.f, 0.f)*scale, p+Vec3(0.f, 1.f, 0.f)*scale,
            p+Vec3( 0.f, 0.f,-1.f)*scale, p+Vec3(0.f, 0.f, 1.f)*scale,
        };
        renderer.drawLines(mvp, markerLines, Rgba(1.f, 0.f, 0.f, 0.f), 0.01f);
    }

    if (mShowDebugWireframe) {
        // Debug wireframe.
        Vector<Point3> debugWireframe;
        debugWireframe.reserve(mTetra.surfaceTriangles.size() * 6);
        for (const Triangle& tri: mTetra.surfaceTriangles) {
            const Point3& p0 = mParticlePositions.at(tri.i0);
            const Point3& p1 = mParticlePositions.at(tri.i1);
            const Point3& p2 = mParticlePositions.at(tri.i2);
            debugWireframe.push_back(p0); debugWireframe.push_back(p1);
            debugWireframe.push_back(p1); debugWireframe.push_back(p2);
            debugWireframe.push_back(p2); debugWireframe.push_back(p0);
        }
        renderer.drawLines(mvp, debugWireframe, Rgba(0.f, 1.f, 0.f, 0.f), 0.005f/mOrbiCam.cameraR);
    }

    mTweakBar.draw();
}
