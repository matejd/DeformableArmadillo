#ifndef Renderer_Hpp
#define Renderer_Hpp

#include "Platform.hpp"

/// Inspired heavily by Emil Persson (Humus, http://www.humus.name) and his framework.
/// This Renderer is a simplified and tweaked version of his code.
/// It's designed with OpenGL ES 2 (and WebGL 1) in mind.

typedef int TextureID;
typedef int ShaderID;
typedef int FramebufferID;
typedef int RenderbufferID;
typedef int IndexBufferID;
typedef int VertexBufferID;
typedef int VertexFormatID;

struct Texture;
struct Shader;
struct Renderbuffer;
struct Framebuffer;
struct IndexBuffer;
struct VertexBuffer;
struct VertexFormat;

enum class PixelFormat {
    R,
    Rg,
    Rgb,
    Rgba,
    Depth16,
    Depth24
};

enum class PixelType {
    Ubyte,
    Float
};

enum class Primitive {
    Triangle,
    Line
};

enum class TextureFilter {
    Nearest,
    Linear,
    Bilinear,
    Trilinear
};

enum class CullMode {
    None,
    Front,
    Back
};

enum class VertexAttribType {
    Float,
    Uint16,
    Uint8,
    Int16,
    Int8
};

struct VertexAttrib
{
    int numComponents;
    VertexAttribType type;
    bool normalized;
    int strideBytes;
    int offsetBytes;
};

class Renderer {
public:
    Renderer();
    ~Renderer();

    TextureID addTexture(const String& filename, const PixelFormat internal, const PixelFormat input, const PixelType, const TextureFilter);
    TextureID addEmptyTexture(const int width, const int height, const PixelFormat internal, const PixelType, const TextureFilter);
    Renderer& setTexture(const int unit, const TextureID);

    ShaderID addShader(const Vector<String>& vsFilenames, const Vector<String>& fsFilenames);
    ShaderID addShaderFromSource(const String& vsSource, const String& fsSource);

    FramebufferID addFramebuffer();
    RenderbufferID addRenderbuffer(const int width, const int height, const PixelFormat format);

    // All previously-void methods return a reference to the instance this method was called on.
    // This enables chaining, which helps with readability (e.g. when setting many uniforms). 
    Renderer& attachTextureToFramebuffer(const FramebufferID framebuffer, const TextureID color);
    Renderer& attachRenderbufferToFramebuffer(const FramebufferID framebuffer, const RenderbufferID renderbuffer);
    Renderer& setFramebuffer(const FramebufferID framebuffer);
    Renderer& setDefaultFramebuffer();

    Renderer& setShader(const ShaderID shader);
    Renderer& setUniform1i(const String& name, const int value);
    Renderer& setUniform1f(const String& name, const float value);
    Renderer& setUniform2fv(const String& name, const int count, const float* const value);
    Renderer& setUniform3fv(const String& name, const int count, const float* const value);
    Renderer& setUniform4fv(const String& name, const int count, const float* const value);
    Renderer& setUniform3x3fv(const String& name, const int count, const float* const value);
    Renderer& setUniform4x4fv(const String& name, const int count, const float* const value);
    // Type-safe variants:
    Renderer& setUniform3f(const String& name, const Vec3& value);
    Renderer& setUniform4f(const String& name, const Vec4& value);
    Renderer& setUniform3x3f(const String& name, const Matrix3& value);
    Renderer& setUniform4x4f(const String& name, const Matrix4& value);
    Renderer& setUniform3fv(const String& name, const Vector<Vec3>& values);
    Renderer& setUniform4fv(const String& name, const Vector<Vec4>& values);
    Renderer& setUniform3x3fv(const String& name, const Vector<Matrix3>& values);
    Renderer& setUniform4x4fv(const String& name, const Vector<Matrix4>& values);

    template <typename INDEX>
    IndexBufferID addIndexBuffer(const Vector<INDEX>& indices)
    {
        STATIC_ASSERT((sizeof(INDEX) == sizeof(u8)) || (sizeof(INDEX) == sizeof(u16)) || (sizeof(INDEX) == sizeof(u32)));
        // Signed index types are also allowed, but are treated as unsigned!
        return addIndexBuffer(indices.data(), sizeof(INDEX), indices.size());
    }

    template <typename VERTEX>
    VertexBufferID addVertexBuffer(const Vector<VERTEX>& vertices)
    {
        return addVertexBuffer(vertices.data(), sizeof(VERTEX) * vertices.size());
    }

    template <typename VERTEX>
    Renderer& updateVertexBuffer(const VertexBufferID id, const Vector<VERTEX>& vertices)
    {
        return updateVertexBuffer(id, vertices.data(), sizeof(VERTEX) * vertices.size());
    }

    VertexFormatID addVertexFormat(const Vector<VertexAttrib>& attribs);

    Renderer& setInputAssembler(const VertexBufferID, const VertexFormatID, const IndexBufferID=-1);
    Renderer& drawIndexedPrimitives(const Primitive, const int firstIndex=0, const int numIndices=-1);
    Renderer& drawPrimitives(const Primitive, const int firstIndex=0, const int numIndices=-1);

    Renderer& clear(const Rgba& color, const float depth=-1.f);
    Renderer& setViewport(const int x, const int y, const int width, const int height);
    Renderer& setCullMode(const CullMode mode);
    Renderer& setDepthWrite(const bool enabled);
    Renderer& setDepthTest(const bool enabled);

    Renderer& drawScreenQuad();
    Renderer& drawLines(const Matrix4& mvp, const Vector<Point3>& vertices, const Rgba& color, const float radius);

    Renderer& liveReloadUpdate();
    Renderer& checkGLError();

private:
    IndexBufferID addIndexBuffer(const void* const data, const int indexSizeBytes, const int numIndices);
    VertexBufferID addVertexBuffer(const void* const data, const int sizeBytes);
    Renderer& updateVertexBuffer(const VertexBufferID id, const void* const data, const int sizeBytes);

    void setTextureFilter(const TextureFilter);
    void checkFramebufferStatus();

    // TODO: no pointers!
    Vector<Texture*> textures;
    Vector<Shader*> shaders;
    Vector<Renderbuffer*> renderbuffers;
    Vector<Framebuffer*> framebuffers;
    Vector<IndexBuffer*> indexBuffers;
    Vector<VertexBuffer*> vertexBuffers;
    Vector<VertexFormat*> vertexFormats;

    struct ShaderTrackingInfo {
        Vector<String> vsFilenames;
        Vector<String> fsFilenames;
        u64 lastModificationTime;
    };
    Map<ShaderID, ShaderTrackingInfo> trackedShaderFiles;
    bool dontAddToTrackedFiles = false;

    ShaderID currentShader = -1;
    IndexBufferID currentIndexBuffer = -1;
    VertexBufferID currentVertexBuffer = -1;
    int viewportX, viewportY, viewportWidth, viewportHeight;

    u32 quadVB = 42;

    ShaderID lineShader = -1;
    VertexBufferID lineVB = -1;
    VertexFormatID lineVF = -1;
};

#endif // Renderer_Hpp
