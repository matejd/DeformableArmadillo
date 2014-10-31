#ifndef Platform_Hpp
#define Platform_Hpp

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <cstdint>
#include <cassert>

#define GLM_SWIZZLE 
#include "external/glm/glm.hpp"

typedef std::int8_t  i8;
typedef std::int16_t i16;
typedef std::int32_t i32;
typedef std::int64_t i64;
typedef std::uint8_t  u8;
typedef std::uint16_t u16;
typedef std::uint32_t u32;
typedef std::uint64_t u64;

const float PI = 3.14159265359f;
const float TwoPI = 2.f * PI;
const float PI2 = PI / 2.f;

typedef glm::vec2 Vec2;
typedef glm::vec3 Vec3;
typedef glm::vec4 Vec4;
typedef glm::vec2 Point2;
typedef glm::vec3 Point3;
typedef glm::vec4 Point4;
typedef glm::mat3 Matrix3;
typedef glm::mat4 Matrix4;

typedef glm::vec3 Rgb;
typedef glm::vec4 Rgba;

template <typename T> using Vector = std::vector<T>;
template <typename T> using Set = std::set<T>;
template <typename K, typename V> using Map = std::map<K, V>;
template <typename T> using Queue = std::queue<T>;
typedef std::string String;

#define STATIC_ASSERT(expr) static_assert(expr, #expr)

#ifdef EMSCRIPTEN
    #define ASSERT(expr)
#else
    #define ASSERT(expr) assert(expr)
#endif

/// This generates a compile time error, e.g. "implicit instantiation of undefined template 'IncompleteType<48>'"
/// giving you the size of your structure.
template <int size> struct IncompleteType;
#define GET_SIZEOF(structure) IncompleteType<sizeof(structure)> incompleteType

#ifndef EMSCRIPTEN
#ifndef NDEBUG
    #define DEBUG
#endif
#endif

template <typename T>
inline void _log_impl(const T& value, const String& exprString)
{
    std::cout << exprString << " = " << value << std::endl;
}

template <>
inline void _log_impl(const Vec3& value, const String& exprString)
{
    std::cout << exprString << " = (" << value.x << " " << value.y << " " << value.z << ")" << std::endl;
}

#define LOG(expr) _log_impl(expr, #expr)

#endif
