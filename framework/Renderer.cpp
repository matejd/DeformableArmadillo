#include "Renderer.hpp"

#include <GL/glew.h>
#include <GL/glfw.h>

#include "external/glm/gtc/type_ptr.hpp"

// stblib image loading library, single-file, public domain
// https://code.google.com/p/stblib/
#define STBI_HEADER_FILE_ONLY
#include "stb_image.cpp"

#include <sstream>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <unordered_map>

#include <sys/stat.h>

using namespace glm;

struct Shader
{
    GLuint id;
    Map<String, GLint> uniforms; // TODO: compile-time string hashing?
};

struct Texture
{
    GLuint id;
    int width, height;
};

struct Framebuffer
{
    GLuint id;
};

struct Renderbuffer
{
    GLuint id;
};

struct IndexBuffer
{
    GLuint id;
    int numIndices;
    int indexSizeBytes;
};

struct VertexBuffer
{
    GLuint id;
};

struct VertexFormat
{
    Vector<VertexAttrib> attribs;
};

String getFileContents(const String& filename)
{
    std::ifstream in(filename, std::ios::in | std::ios::binary);
    if (in) {
        String contents;
        in.seekg(0, std::ios::end);
        contents.resize(in.tellg());
        in.seekg(0, std::ios::beg);
        in.read(&contents[0], contents.size());
        in.close();
        return contents;
    }
    else {
        std::cout << "Failed to read " << filename << "!" << std::endl;
        ASSERT(false);
        return String();
    }
}

u64 getFileModificationTime(const String& filename)
{
    struct stat statInfo;
    int result = stat(filename.c_str(), &statInfo);
    if (result < 0) {
        std::cout << "result: " << result << std::endl;
        ASSERT(result >= 0);
    }
    time_t time = statInfo.st_mtime;
    return time;
}

Renderer& Renderer::checkGLError()
{
    const GLenum error = glGetError();
    if (error == GL_NO_ERROR)
        return *this;
    std::cout << "OpenGL error: ";
    switch (error) {
        case GL_INVALID_ENUM:      std::cout << "invalid enum"; break;
        case GL_INVALID_VALUE:     std::cout << "invalid value"; break;
        case GL_INVALID_OPERATION: std::cout << "invalid operation"; break;
        case GL_STACK_OVERFLOW:    std::cout << "stack overflow"; break;
        case GL_OUT_OF_MEMORY:     std::cout << "out of memory"; break;
        case GL_TABLE_TOO_LARGE:   std::cout << "table too large"; break;
        default:                   std::cout << "unknown"; break;
    }
    std::cout << std::endl;
    ASSERT(false);
    return *this;
}

Renderer::Renderer()
{
}

Renderer::~Renderer()
{
    // TODO: this is silly!
    //
    for (Shader* shader: shaders) {
        delete shader;
    }

    for (Texture* texture: textures) {
        delete texture;
    }

    for (Renderbuffer* renderbuffer: renderbuffers) {
        delete renderbuffer;
    }

    for (Framebuffer* framebuffer: framebuffers) {
        delete framebuffer;
    }

    for (IndexBuffer* buffer: indexBuffers) {
        delete buffer;
    }

    for (VertexBuffer* buffer: vertexBuffers) {
        delete buffer;
    }

    for (VertexFormat* format: vertexFormats) {
        delete format;
    }
}

ShaderID Renderer::addShaderFromSource(const String& vsSource, const String& fsSource)
{
    ASSERT(fsSource.size() > 0);
    String vsSourceFinal = vsSource;
    if (vsSource.size() == 0) {
        // When no vertex shader is specified, the user wants
        // a simple pass-through shader (think Shadertoy fullscreen shaders).
        // See drawScreenQuad() for more info.
        vsSourceFinal = String("attribute float index;\n") +
                        "uniform vec2 uv[4];\n" +
                        "varying vec2 vuv;\n" +
                        "void main(){\n" +
                        "int iindex = int(index);\n" +
                        "vuv = uv[iindex];\n" +
                        "gl_Position = vec4(vuv*2.0-1.0, 0.5, 1.0);}";
    }
    const String fsHeader = String("#if GL_ES\n") +
        "#ifdef GL_FRAGMENT_PRECISION_HIGH\n" +
        "precision highp float;\n" +
        "#else\n" +
        "precision mediump float;\n" +
        "#endif\n" +
        "#endif\n";
    const String fsSourceFinal = fsHeader + fsSource;
    const GLenum types[] = {GL_VERTEX_SHADER, GL_FRAGMENT_SHADER};
    GLuint ids[2];
    for (int i = 0; i < 2; i++) {
        ids[i] = glCreateShader(types[i]);
        const char* ptr = (i == 0) ? &vsSourceFinal[0] : &fsSourceFinal[0];
        glShaderSource(ids[i], 1, &ptr, nullptr);
        glCompileShader(ids[i]);
        GLint status = 0;
        glGetShaderiv(ids[i], GL_COMPILE_STATUS, &status);
        if (!status) {
            GLchar info[1024];
            GLsizei length = 0;
            glGetShaderInfoLog(ids[i], sizeof(info), &length, info);
            std::cout << "Failed to compile:" << std::endl << info << std::endl;
            ASSERT(false);
            return -1;
        }
    }

    // Parse vertex and fragment shader, extract attribute and uniform
    // names. This parsing might fail, should probably be improved
    // someday.
    Vector<String> attributes;
    Vector<String> uniforms;
    for (int i = 0; i < 2; i++) {
        std::stringstream ss;
        if (i == 0)
            ss << vsSourceFinal;
        else
            ss << fsSourceFinal;
        ss.seekp(0);
        String token;
        ss >> token;
        while (token != "main" && !ss.eof()) {
            if (token == "uniform") {
                String type, name;
                ss >> type >> name;
                name = name.substr(0, name.find_first_of("[ ;"));
                uniforms.push_back(name);
            }
            else if (token == "attribute") {
                String type, name;
                ss >> type >> name;
                name = name.substr(0, name.find_first_of("[ ;"));
                attributes.push_back(name);
            }

            ss >> token;
        }
    }

    Shader* shader = new Shader;
    shader->id = glCreateProgram();
    glAttachShader(shader->id, ids[0]);
    glAttachShader(shader->id, ids[1]);
    for (int i = 0; i < attributes.size(); i++) {
        glBindAttribLocation(shader->id, i, attributes[i].c_str());
    }
    glLinkProgram(shader->id);
    GLint linked = 0;
    glGetProgramiv(shader->id, GL_LINK_STATUS, &linked);
    ASSERT(linked);
    for (String name : uniforms) {
        shader->uniforms[name] = glGetUniformLocation(shader->id, name.c_str());
        if (shader->uniforms[name] == -1) {
            std::cout << "Failed to get uniform " << name << " location!" << std::endl;
            ASSERT(false);
        }
    }

    shaders.push_back(shader);
    return shaders.size()-1;
}

ShaderID Renderer::addShader(const Vector<String>& vsFilenames, const Vector<String>& fsFilenames)
{
    std::stringstream ss;
    ss << "Uploading shaders: ";
    std::copy(vsFilenames.begin(), vsFilenames.end(), std::ostream_iterator<String>(ss, " "));
    ss << "+ ";
    std::copy(fsFilenames.begin(), fsFilenames.end(), std::ostream_iterator<String>(ss, " "));
    std::cout << ss.str() << std::endl;

    String vsSource = "";
    String fsSource = "";
    for (const String& file: vsFilenames) {
        vsSource += getFileContents(file) + "\n";
    }
    for (const String& file: fsFilenames) {
        fsSource += getFileContents(file) + "\n";
    }
    const ShaderID id = addShaderFromSource(vsSource, fsSource);
    if (!dontAddToTrackedFiles) {
        ShaderTrackingInfo info;
        info.vsFilenames = vsFilenames;
        info.fsFilenames = fsFilenames;
        u64 modTime = 0;
        for (const String& name: vsFilenames) {
            modTime = std::max(modTime, getFileModificationTime(name));
        }
        for (const String& name: fsFilenames) {
            modTime = std::max(modTime, getFileModificationTime(name));
        }
        info.lastModificationTime = modTime;
        trackedShaderFiles[id] = info;
    }
    return id;
}

Renderer& Renderer::setShader(const ShaderID shader)
{
    glUseProgram(shaders.at(shader)->id);
    currentShader = shader;
    return *this;
}

Renderer& Renderer::setUniform1i(const String& name, const int value)
{
    Shader* shader = shaders.at(currentShader);
    glUniform1i(shader->uniforms.at(name), value);
    return *this;
}

Renderer& Renderer::setUniform1f(const String& name, const float value)
{
    Shader* shader = shaders.at(currentShader);
    glUniform1f(shader->uniforms.at(name), value);
    return *this;
}

Renderer& Renderer::setUniform3x3fv(const String& name, const int count, const float* const value)
{
    Shader* shader = shaders.at(currentShader);
    glUniformMatrix3fv(shader->uniforms.at(name), count, GL_FALSE, value);
    return *this;
}

Renderer& Renderer::setUniform4x4fv(const String& name, const int count, const float* const value)
{
    Shader* shader = shaders.at(currentShader);
    glUniformMatrix4fv(shader->uniforms.at(name), count, GL_FALSE, value);
    return *this;
}

Renderer& Renderer::setUniform3fv(const String& name, int count, const float* const value)
{
    Shader* shader = shaders.at(currentShader);
    glUniform3fv(shader->uniforms.at(name), count, value);
    return *this;
}

Renderer& Renderer::setUniform4fv(const String& name, int count, const float* const value)
{
    Shader* shader = shaders.at(currentShader);
    glUniform4fv(shader->uniforms.at(name), count, value);
    return *this;
}

Renderer& Renderer::setUniform2fv(const String& name, int count, const float* const value)
{
    Shader* shader = shaders.at(currentShader);
    glUniform2fv(shader->uniforms.at(name), count, value);
    return *this;
}

Renderer& Renderer::setUniform3f(const String& name, const Vec3& value)
{
    Shader* shader = shaders.at(currentShader);
    glUniform3f(shader->uniforms.at(name), value.x, value.y, value.z);
    return *this;
}

Renderer& Renderer::setUniform4f(const String& name, const Vec4& value)
{
    Shader* shader = shaders.at(currentShader);
    glUniform4f(shader->uniforms.at(name), value.x, value.y, value.z, value.w);
    return *this;
}

Renderer& Renderer::setUniform3x3f(const String& name, const Matrix3& value)
{
    Shader* shader = shaders.at(currentShader);
    glUniformMatrix3fv(shader->uniforms.at(name), 1, GL_FALSE, glm::value_ptr(value));
    return *this;
}

Renderer& Renderer::setUniform4x4f(const String& name, const Matrix4& value)
{
    Shader* shader = shaders.at(currentShader);
    glUniformMatrix4fv(shader->uniforms.at(name), 1, GL_FALSE, glm::value_ptr(value));
    return *this;
}

Renderer& Renderer::setUniform3fv(const String& name, const Vector<Vec3>& values)
{
    Shader* shader = shaders.at(currentShader);
    glUniform3fv(shader->uniforms.at(name), values.size(), glm::value_ptr(values.at(0)));
    return *this;
}

Renderer& Renderer::setUniform4fv(const String& name, const Vector<Vec4>& values)
{
    Shader* shader = shaders.at(currentShader);
    glUniform4fv(shader->uniforms.at(name), values.size(), glm::value_ptr(values.at(0)));
    return *this;
}

Renderer& Renderer::setUniform3x3fv(const String& name, const Vector<Matrix3>& values)
{
    Shader* shader = shaders.at(currentShader);
    glUniformMatrix3fv(shader->uniforms.at(name), values.size(), GL_FALSE, glm::value_ptr(values.at(0)));
    return *this;
}

Renderer& Renderer::setUniform4x4fv(const String& name, const Vector<Matrix4>& values)
{
    Shader* shader = shaders.at(currentShader);
    glUniformMatrix4fv(shader->uniforms.at(name), values.size(), GL_FALSE, glm::value_ptr(values.at(0)));
    return *this;
}

Renderer& Renderer::drawScreenQuad()
{
    if (quadVB == 42) {
        const float quadIndices[] = {
            0.f, 1.f, 2.f,
            0.f, 2.f, 3.f
        };

        glGenBuffers(1, &quadVB);
        glBindBuffer(GL_ARRAY_BUFFER, quadVB);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quadIndices), quadIndices, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    const float uv[] = {
        0.f, 0.f,
        1.f, 0.f,
        1.f, 1.f,
        0.f, 1.f
    };

    setUniform2fv("uv", 4, uv);
    glBindBuffer(GL_ARRAY_BUFFER, quadVB);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, 1*sizeof(float), reinterpret_cast<GLvoid*>(0));
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glDisableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return *this;
}

Renderer& Renderer::drawLines(const Matrix4& mvp, const Vector<Point3>& vertices, const Rgba& color, const float radius)
{
    /// Two vertices specify a line (like GL_LINES in the old OpenGL).
    /// Each line gets rendered as a view-plane-facing
    /// quad (two triangles). Depth is properly respected.
    /// radius is screespace half-width of the line (radius value 0.5 would fill the screen vertically).
    /// This function changes blend state and depth mask state!

    ASSERT(vertices.size() % 2 == 0);
    const int numLines = vertices.size() / 2;
    const int kMaxLinesPerBatch = 256; // More than this many lines means multiple draw calls.

    if (lineShader == -1) {
        Vector<float> indices;
        indices.reserve(kMaxLinesPerBatch * 6);
        for (int line = 0; line < kMaxLinesPerBatch; line++) {
            const int i = line*4; // First index.
            // Two triangles.
            indices.push_back(static_cast<float>(i+0));
            indices.push_back(static_cast<float>(i+1));
            indices.push_back(static_cast<float>(i+3));

            indices.push_back(static_cast<float>(i+1));
            indices.push_back(static_cast<float>(i+2));
            indices.push_back(static_cast<float>(i+3));
        }
        ASSERT(indices.size() == kMaxLinesPerBatch*6);
        lineVB = addVertexBuffer(indices); // Yes, vertex buffer.
        lineVF = addVertexFormat({{1, VertexAttribType::Float, false, sizeof(float), 0}});
        lineShader = addShaderFromSource(
            String() +
            "attribute float index;\n" + // Each line has 6 indices (two triangles).
            "uniform vec4 positionsClip[" + std::to_string(kMaxLinesPerBatch*4) + "];\n" + // Each line has 4 distinct vertices.
            "varying float vAlpha;\n" +
            "void main() {\n" +
            "  int ii = int(index);\n" +
            "  int rem = int(index - floor(index/4.0)*4.0);\n" + // index % 4
            "  vAlpha = -1.0;\n" + // Transparency goes to zero at both edges of the line. Line is fully opaque in the middle.
            "  if (rem == 0 || rem == 3) vAlpha = 1.0;\n" +
            "  gl_Position = positionsClip[ii];}\n",
            String() +
            "varying float vAlpha;\n" +
            "uniform vec3 color;\n" +
            "void main() { gl_FragColor = vec4(color, pow(1.0-abs(vAlpha), 0.5)); }\n");
    }

    const Vec2 viewScale(viewportWidth, viewportHeight);

    // TODO: instead of projecting to window space, doing computations there, and
    // then transforming back to clip space, think about doing computations directly
    // in clip space!
    Vector<Point3> positionsScreen;
    positionsScreen.reserve(numLines * 4);
    for (size_t i = 0; i < numLines*2; i += 2) {
        const Point4 clip0 = mvp * Point4(vertices[i+0], 1.f);
        const Point4 clip1 = mvp * Point4(vertices[i+1], 1.f);
        const Point2 clip0xy = clip0.xy();
        const Point2 clip1xy = clip1.xy();
        const Point2 screen0 = ((clip0xy / clip0.w)*0.5f + Vec2(0.5f, 0.5f))*viewScale;
        const Point2 screen1 = ((clip1xy / clip1.w)*0.5f + Vec2(0.5f, 0.5f))*viewScale;
        const Vec2 dir = normalize(screen1-screen0);
        const Vec2 ortho(-dir.y, dir.x);
        const float rad = (viewportHeight/2.f) * radius;
        positionsScreen.push_back(Point3(screen0 - rad*dir + rad*ortho, clip0.z/clip0.w));
        positionsScreen.push_back(Point3(screen0 - rad*dir - rad*ortho, clip0.z/clip0.w));
        positionsScreen.push_back(Point3(screen1 + rad*dir - rad*ortho, clip1.z/clip1.w));
        positionsScreen.push_back(Point3(screen1 + rad*dir + rad*ortho, clip1.z/clip1.w));
    }
    ASSERT(positionsScreen.size() == numLines*4);

    // Back to clip space, keep z what it was (already in normalized device coordinates), but set w to 1.
    // Come to think of it, this is actually already in NDC, since w is 1 and perspective division
    // changes nothing.
    Vector<Point4> positionsClip;
    positionsClip.reserve(positionsScreen.size());
    for (const Point3& pS: positionsScreen) {
        positionsClip.push_back(Point4(((pS.xy() / viewScale) - Vec2(0.5f, 0.5f))*2.f, pS.z, 1.f));
    }

    glDepthMask(GL_FALSE); // Depth test can be enabled, but don't write new depths!
    glEnable(GL_BLEND);
    glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ZERO);

    setShader(lineShader);
    setUniform3fv("color", 1, reinterpret_cast<const float*>(&color));
    setInputAssembler(lineVB, lineVF);

    const int numIndices = numLines * 6;
    int numRemainingIndices = numIndices;
    const int numPositions = numLines * 4;
    int numRemainingPositions = numPositions;

    for (int line = 0; line < numLines; line += kMaxLinesPerBatch) {
        const int numIndicesInBatch   = min(kMaxLinesPerBatch*6, numRemainingIndices);
        const int numPositionsInBatch = min(kMaxLinesPerBatch*4, numRemainingPositions);
        numRemainingIndices -= numIndicesInBatch;
        numRemainingPositions -= numPositionsInBatch;
        setUniform4fv("positionsClip", numPositionsInBatch, reinterpret_cast<const float*>(&positionsClip[line*4]));
        drawPrimitives(Primitive::Triangle, 0, numIndicesInBatch);
    }
    ASSERT(numRemainingIndices == 0);
    ASSERT(numRemainingPositions == 0);

    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    return *this;
}

TextureID Renderer::addEmptyTexture(const int width, const int height, const PixelFormat format, const PixelType type, const TextureFilter filter)
{
    int numChannels = 1;
    GLenum glFormat = GL_RED;
    switch (format) {
        case PixelFormat::R:    numChannels = 1; glFormat = GL_RED;  break;
        case PixelFormat::Rg:   numChannels = 2; glFormat = GL_RG;   break;
        case PixelFormat::Rgb:  numChannels = 3; glFormat = GL_RGB;  break;
        case PixelFormat::Rgba: numChannels = 4; glFormat = GL_RGBA; break;
        default: ASSERT(false);
    };

    GLenum glType = GL_UNSIGNED_BYTE;
    switch (type) {
        case PixelType::Float: glType = GL_FLOAT;         break;
        case PixelType::Ubyte: glType = GL_UNSIGNED_BYTE; break;
        default: ASSERT(false);
    };

    Texture* tex = new Texture;
    glGenTextures(1, &tex->id);
    glBindTexture(GL_TEXTURE_2D, tex->id);
#ifdef EMSCRIPTEN
    // WebGL supported formats: ALPHA, RGB, RGBA, LUMINANCE, LUMINANCE_ALPHA.
    // Internal format must match input format (no conversion is done).
    // Type float is an extension!
    // Single channel textures are not renderable!
    if (glFormat == GL_RED)
        glFormat = GL_LUMINANCE;
    glTexImage2D(GL_TEXTURE_2D, 0, glFormat, width, height, 0, glFormat, glType, nullptr);
#else
    GLenum glInternal = glFormat;
    if (type == PixelType::Float) {
        switch (format) {
            case PixelFormat::R:   glInternal = GL_R32F;   break;
            case PixelFormat::Rg:  glInternal = GL_RG32F;  break;
            case PixelFormat::Rgb: glInternal = GL_RGB32F; break;
            default: ASSERT(false);
        }
    }
    glTexImage2D(GL_TEXTURE_2D, 0, glInternal, width, height, 0, glFormat, glType, nullptr);
#endif

    setTextureFilter(filter);
    textures.push_back(tex);
    return textures.size()-1;
}

TextureID Renderer::addTexture(const String& filename, const PixelFormat internal, const PixelFormat input, const PixelType type, const TextureFilter filter)
{
    int numChannels = 1;
    GLenum glInternal = GL_RED;
    switch (input) {
        case PixelFormat::R:    numChannels = 1; glInternal = GL_RED;  break;
        case PixelFormat::Rg:   numChannels = 2; glInternal = GL_RG;   break;
        case PixelFormat::Rgb:  numChannels = 3; glInternal = GL_RGB;  break;
        case PixelFormat::Rgba: numChannels = 4; glInternal = GL_RGBA; break;
        default: ASSERT(false);
    };

    ASSERT(internal == input);
    GLenum glInput = glInternal;

    GLenum glType;
    switch (type) {
        case PixelType::Float: glType = GL_FLOAT;         break;
        case PixelType::Ubyte: glType = GL_UNSIGNED_BYTE; break;
        default: ASSERT(false);
    };

    int width, height, n;
    u8* data = nullptr;
    // Delegate all the hard work to the fantastic stb_image.
    data = stbi_load(filename.c_str(), &width, &height, &n, numChannels);
    if (data == nullptr) {
        LOG(stbi_failure_reason());
        ASSERT(false);
    }
    ASSERT(n == numChannels);

    Texture* tex = new Texture;
    tex->width = width;
    tex->height = height;
    glGenTextures(1, &tex->id);
    glBindTexture(GL_TEXTURE_2D, tex->id);
    glTexImage2D(GL_TEXTURE_2D, 0, glInternal, width, height, 0, glInput, glType, data);
    setTextureFilter(filter);
    stbi_image_free(data);
    textures.push_back(tex);
    return textures.size()-1;
}

void Renderer::setTextureFilter(const TextureFilter filter)
{
    GLint minFilter, magFilter;
    switch (filter) {
        case TextureFilter::Nearest:   minFilter = GL_NEAREST;                magFilter = GL_NEAREST; break;
        case TextureFilter::Linear:    minFilter = GL_LINEAR;                 magFilter = GL_LINEAR; break;
        case TextureFilter::Bilinear:  minFilter = GL_LINEAR_MIPMAP_NEAREST;  magFilter = GL_LINEAR; break;
        case TextureFilter::Trilinear: minFilter = GL_LINEAR_MIPMAP_LINEAR;   magFilter = GL_LINEAR; break;
    };

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

Renderer& Renderer::setTexture(const int unit, const TextureID id)
{
    ASSERT(unit >= 0);
    Texture* texture = textures.at(id);
    glActiveTexture(GL_TEXTURE0+unit);
    glBindTexture(GL_TEXTURE_2D, texture->id);
    return *this;
}

FramebufferID Renderer::addFramebuffer()
{
    Framebuffer* framebuffer = new Framebuffer;
    glGenFramebuffers(1, &framebuffer->id);
    framebuffers.push_back(framebuffer);
    checkGLError();
    return framebuffers.size()-1;
}

RenderbufferID Renderer::addRenderbuffer(const int width, const int height, const PixelFormat format)
{
    Renderbuffer* renderbuffer = new Renderbuffer;
    glGenRenderbuffers(1, &renderbuffer->id);
    glBindRenderbuffer(GL_RENDERBUFFER, renderbuffer->id);
    ASSERT(format == PixelFormat::Depth16);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, width, height);
    renderbuffers.push_back(renderbuffer);
    checkGLError();
    return renderbuffers.size()-1;
}

Renderer& Renderer::attachTextureToFramebuffer(const FramebufferID framebuffer, const TextureID color)
{
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffers.at(framebuffer)->id);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textures.at(color)->id, 0);
    checkFramebufferStatus();
    checkGLError();
    return *this;
}

Renderer& Renderer::attachRenderbufferToFramebuffer(const FramebufferID framebuffer, const RenderbufferID buffer)
{
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffers.at(framebuffer)->id);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, renderbuffers.at(buffer)->id);
    checkFramebufferStatus();
    checkGLError();
    return *this;
}

void Renderer::checkFramebufferStatus()
{
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status == GL_FRAMEBUFFER_COMPLETE)
        return;

    String error;
    switch (status) {
        case GL_FRAMEBUFFER_UNSUPPORTED:                   error = "Framebuffer unsupported!"; break;
        case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:         error = "Framebuffer error: incomplete attachment!"; break;
        case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:     error = "Framebuffer error: incomplete dimensions!"; break;
        case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT: error = "Framebuffer error: missing attachment!"; break;
        default: error = "Framebuffer error: unknown!"; break;
    };
    std::cout << error << std::endl;
}

Renderer& Renderer::setDefaultFramebuffer()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return *this;
}

Renderer& Renderer::setFramebuffer(const FramebufferID framebuffer)
{
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffers.at(framebuffer)->id);
    return *this;
}

Renderer& Renderer::setViewport(const int x, const int y, const int width, const int height)
{
    glViewport(x, y, width, height);
    viewportX = x;
    viewportY = y;
    viewportWidth = width;
    viewportHeight = height;
    return *this;
}

//IndexBufferID Renderer::addIndexBuffer(const Vector<unsigned int>& indices)
IndexBufferID Renderer::addIndexBuffer(const void* const data, const int indexSizeBytes, const int numIndices)
{
    IndexBuffer* indexBuffer = new IndexBuffer;
    indexBuffer->numIndices = numIndices;//indices.size();
    indexBuffer->indexSizeBytes = indexSizeBytes;
    glGenBuffers(1, &indexBuffer->id);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer->id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, numIndices * indexSizeBytes, data, GL_STATIC_DRAW);//indices.size() * sizeof(indices.at(0)), indices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    indexBuffers.push_back(indexBuffer);
    checkGLError();
    return indexBuffers.size()-1;
}

VertexBufferID Renderer::addVertexBuffer(const void* const data, const int sizeBytes)
{
    VertexBuffer* vertexBuffer = new VertexBuffer;
    glGenBuffers(1, &vertexBuffer->id);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer->id);
    glBufferData(GL_ARRAY_BUFFER, sizeBytes, data, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    vertexBuffers.push_back(vertexBuffer);
    checkGLError();
    return vertexBuffers.size()-1;
}

Renderer& Renderer::updateVertexBuffer(const VertexBufferID id, const void* const data, const int sizeBytes)
{
    const VertexBuffer* vertexBuffer = vertexBuffers.at(id);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer->id);
    glBufferData(GL_ARRAY_BUFFER, sizeBytes, data, GL_STATIC_DRAW); // TODO: GL_DYNAMIC?
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    checkGLError();
    return *this;
}

VertexFormatID Renderer::addVertexFormat(const Vector<VertexAttrib>& attribs)
{
    VertexFormat* vertexFormat = new VertexFormat;
    vertexFormat->attribs = attribs;
    vertexFormats.push_back(vertexFormat);
    return vertexFormats.size()-1;
}

Renderer& Renderer::setInputAssembler(const VertexBufferID vb, const VertexFormatID vf, const IndexBufferID ib)
{
    const VertexBuffer* vertexBuffer = vertexBuffers.at(vb);;
    currentVertexBuffer = vb;
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer->id);

    const VertexFormat* vertexFormat = vertexFormats.at(vf);

    /// The fact that we need to make sure higher vertex attributes are
    /// disabled at this point probably indicates a bad Renderer design!
    for (int i = 0; i < 4; i++)
        glDisableVertexAttribArray(i);

    int index = 0;
    for (const VertexAttrib& attrib: vertexFormat->attribs) {
        glEnableVertexAttribArray(index);
        GLenum type;
        switch (attrib.type) {
            case VertexAttribType::Float:  type = GL_FLOAT;          break;
            case VertexAttribType::Uint16: type = GL_UNSIGNED_SHORT; break;
            case VertexAttribType::Uint8:  type = GL_UNSIGNED_BYTE;  break;
            case VertexAttribType::Int16:  type = GL_SHORT;          break;
            case VertexAttribType::Int8:   type = GL_BYTE;           break;
            default:
                ASSERT(false);
        };
        glVertexAttribPointer(index, attrib.numComponents, type, attrib.normalized,
                attrib.strideBytes, reinterpret_cast<GLvoid*>(attrib.offsetBytes));
        index++;
    }

    if (ib != -1) {
        IndexBuffer* indexBuffer = indexBuffers.at(ib);
        currentIndexBuffer = ib;
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer->id);
    }
    return *this;
}

Renderer& Renderer::drawIndexedPrimitives(const Primitive primitive, const int firstIndex, const int numIndices)
{
    GLenum mode;
    switch (primitive) {
        case Primitive::Triangle: mode = GL_TRIANGLES; break;
        case Primitive::Line:     mode = GL_LINES;     break;
        default:
            ASSERT(false);
    };

    const IndexBuffer* indexBuffer = indexBuffers.at(currentIndexBuffer);
    int finalNumIndices = numIndices;
    if (numIndices == -1) {
        finalNumIndices = indexBuffer->numIndices - firstIndex; // Till the end of the index buffer.
    }

    GLenum indexType = GL_UNSIGNED_BYTE;
    switch (indexBuffer->indexSizeBytes) {
        case sizeof(u8):  indexType = GL_UNSIGNED_BYTE;  break; // TODO: slow?
        case sizeof(u16): indexType = GL_UNSIGNED_SHORT; break;
        case sizeof(u32): indexType = GL_UNSIGNED_INT;   break;
        default:
            ASSERT(false);
    };
    glDrawElements(mode, finalNumIndices, indexType, reinterpret_cast<GLvoid*>(firstIndex));
    return *this;
}

Renderer& Renderer::drawPrimitives(const Primitive primitive, const int firstIndex, const int numIndices)
{
    GLenum mode;
    switch (primitive) {
        case Primitive::Triangle: mode = GL_TRIANGLES; break;
        case Primitive::Line:     mode = GL_LINES;     break;
        default:
            ASSERT(false);
    };

    ASSERT(numIndices != -1);
    const int finalNumIndices = numIndices; // TODO: figure out numIndices from size of the current vertex buffer.
    glDrawArrays(mode, firstIndex, finalNumIndices);
    return *this;
}

Renderer& Renderer::clear(const Rgba& color, const float depth)
{
    glClearColor(color.r, color.g, color.b, color.a);
    GLbitfield mask = GL_COLOR_BUFFER_BIT;
    if (depth >= 0.f && depth <= 1.f) {
        mask |= GL_DEPTH_BUFFER_BIT;
        glClearDepth(depth);
    }
    glClear(mask);
    return *this;
}

Renderer& Renderer::setCullMode(const CullMode mode)
{
    if (mode == CullMode::None) {
        glDisable(GL_CULL_FACE);
        return *this;
    }

    GLenum glMode = GL_BACK;
    switch (mode) {
        case CullMode::Front: glMode = GL_FRONT; break;
        case CullMode::Back:  glMode = GL_BACK;  break;
        default:
            ASSERT(false);
    }
    glCullFace(glMode);
    glEnable(GL_CULL_FACE);
    return *this;
}

Renderer& Renderer::setDepthWrite(const bool enabled)
{
    glDepthMask(enabled);
    return *this;
}

Renderer& Renderer::setDepthTest(const bool enabled)
{
    if (enabled)
        glEnable(GL_DEPTH_TEST);
    else
        glDisable(GL_DEPTH_TEST);
    return *this;
}

Renderer& Renderer::liveReloadUpdate()
{
#ifndef EMSCRIPTEN
    dontAddToTrackedFiles = true;
    for (auto it = trackedShaderFiles.begin(); it != trackedShaderFiles.end(); ++it) {
        const ShaderID id        = it->first;
        ShaderTrackingInfo& info = it->second;
        u64 modTime = 0;
        for (const String& name: info.vsFilenames) {
            modTime = std::max(modTime, getFileModificationTime(name));
        }
        for (const String& name: info.fsFilenames) {
            modTime = std::max(modTime, getFileModificationTime(name));
        }

        if (modTime > info.lastModificationTime) {
            info.lastModificationTime = modTime;

            const ShaderID newId = addShader(info.vsFilenames, info.fsFilenames);
            if (newId != -1) {
                Shader* previousVersion = shaders[id];
                delete previousVersion;
                shaders[id] = shaders[newId];
                shaders.pop_back();
            }
        }
    }
    dontAddToTrackedFiles = false;
#endif
    return *this;
}
