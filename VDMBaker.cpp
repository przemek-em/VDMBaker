// Standalone Win32 Vector Displacement Map baker.
//
// Build from a "x64 Native Tools Command Prompt for VS":
//   cl /EHsc /O2 /std:c++17 /DUNICODE /D_UNICODE VDMBaker.cpp /link /SUBSYSTEM:WINDOWS User32.lib Gdi32.lib Comdlg32.lib Comctl32.lib
//
// No third-party libraries are required. The file contains:
//   - Wavefront OBJ position/UV/face parser
//   - single-mesh exact UV XYZ VDM bake
//   - two-mesh rest/base -> target/detail VDM bake
//   - small triangle BVH for different-topology target projection
//   - uncompressed RGBA32F OpenEXR writer
//   - standard Win32 GUI

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <windows.h>
#include <commctrl.h>
#include <commdlg.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#pragma comment(lib, "Comdlg32.lib")
#pragma comment(lib, "Comctl32.lib")
#pragma comment(lib, "Gdi32.lib")
#pragma comment(lib, "User32.lib")

namespace fs = std::filesystem;

namespace {

constexpr UINT WM_BAKE_PROGRESS = WM_APP + 1;
constexpr UINT WM_BAKE_DONE = WM_APP + 2;

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    Vec2() = default;
    Vec2(float xx, float yy) : x(xx), y(yy) {}
};

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
    explicit Vec3(float v) : x(v), y(v), z(v) {}
};

inline Vec2 operator+(const Vec2& a, const Vec2& b) { return Vec2(a.x + b.x, a.y + b.y); }
inline Vec2 operator-(const Vec2& a, const Vec2& b) { return Vec2(a.x - b.x, a.y - b.y); }
inline Vec2 operator*(const Vec2& a, float s) { return Vec2(a.x * s, a.y * s); }
inline Vec2 operator*(float s, const Vec2& a) { return a * s; }
inline Vec2 operator/(const Vec2& a, float s) { return Vec2(a.x / s, a.y / s); }

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return Vec3(a.x + b.x, a.y + b.y, a.z + b.z); }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return Vec3(a.x - b.x, a.y - b.y, a.z - b.z); }
inline Vec3 operator*(const Vec3& a, float s) { return Vec3(a.x * s, a.y * s, a.z * s); }
inline Vec3 operator*(float s, const Vec3& a) { return a * s; }
inline Vec3 operator/(const Vec3& a, float s) { return Vec3(a.x / s, a.y / s, a.z / s); }

inline float Dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
inline float Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 Cross(const Vec3& a, const Vec3& b) {
    return Vec3(a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x);
}
inline Vec2 Min(const Vec2& a, const Vec2& b) { return Vec2(std::min(a.x, b.x), std::min(a.y, b.y)); }
inline Vec2 Max(const Vec2& a, const Vec2& b) { return Vec2(std::max(a.x, b.x), std::max(a.y, b.y)); }
inline Vec3 Min(const Vec3& a, const Vec3& b) {
    return Vec3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
}
inline Vec3 Max(const Vec3& a, const Vec3& b) {
    return Vec3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
}

struct Aabb {
    Vec3 min{std::numeric_limits<float>::max()};
    Vec3 max{std::numeric_limits<float>::lowest()};

    void Expand(const Vec3& p) {
        min = Min(min, p);
        max = Max(max, p);
    }

    void Expand(const Aabb& b) {
        min = Min(min, b.min);
        max = Max(max, b.max);
    }
};

struct VDMPixel {
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    float a = 0.0f;
};

enum class BakeMode {
    SingleExactUv = 0,
    TwoMesh = 1,
};

enum class BasePlane {
    XY = 0,
    XZ = 1,
    ZY = 2,
};

enum class VectorMode {
    Displacement = 0,
    ObjectPosition = 1,
};

enum class AxisTransform {
    None = 0,
    SwapYZ = 1,
    BlenderObjDefault = 2,
};

struct BakeSettings {
    BakeMode mode = BakeMode::SingleExactUv;
    int width = 1024;
    int height = 1024;
    BasePlane basePlane = BasePlane::XY;
    VectorMode vectorMode = VectorMode::Displacement;
    AxisTransform axisTransform = AxisTransform::BlenderObjDefault;
    bool normalizeObjUVs = false;
    bool flipV = false;
    int edgePadding = 4;
    float baseAxisValue = 0.0f;
    float outputScale = 1.0f;
};

struct BakeResult {
    bool success = false;
    std::string message;
    int width = 0;
    int height = 0;
    size_t coveredPixels = 0;
    size_t triangleCount = 0;
};

struct ObjMesh {
    struct Corner {
        uint32_t position = 0;
        uint32_t uv = 0;
        bool hasUV = false;
    };

    struct Triangle {
        Corner c[3];
    };

    std::vector<Vec3> positions;
    std::vector<Vec2> uvs;
    std::vector<Triangle> triangles;

    bool HasAnyUVs() const {
        for (const Triangle& t : triangles) {
            if (t.c[0].hasUV || t.c[1].hasUV || t.c[2].hasUV) return true;
        }
        return false;
    }
};

struct PreparedTri {
    Vec3 p[3];
    Vec2 uv[3];
};

struct TwoMeshPreparedTri {
    Vec3 restP[3];
    Vec3 targetP[3];
    Vec2 uv[3];
};

struct ChannelStats {
    Vec3 min{std::numeric_limits<float>::max()};
    Vec3 max{std::numeric_limits<float>::lowest()};
    size_t samples = 0;
};

struct ProgressMsg {
    int percent = 0;
    std::string text;
};

struct DoneMsg {
    bool ok = false;
    std::string text;
};

using ProgressFn = std::function<void(float, const std::string&)>;

inline bool Cancelled(std::atomic<bool>* c) {
    return c && c->load(std::memory_order_relaxed);
}

inline void Report(ProgressFn cb, float value, const std::string& text) {
    if (cb) cb(std::clamp(value, 0.0f, 1.0f), text);
}

std::wstring Utf8ToWide(const std::string& s) {
    if (s.empty()) return std::wstring();
    int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), static_cast<int>(s.size()), nullptr, 0);
    std::wstring out(static_cast<size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), static_cast<int>(s.size()), out.data(), n);
    return out;
}

std::string WideToUtf8(const std::wstring& s) {
    if (s.empty()) return std::string();
    int n = WideCharToMultiByte(CP_UTF8, 0, s.c_str(), static_cast<int>(s.size()), nullptr, 0, nullptr, nullptr);
    std::string out(static_cast<size_t>(n), '\0');
    WideCharToMultiByte(CP_UTF8, 0, s.c_str(), static_cast<int>(s.size()), out.data(), n, nullptr, nullptr);
    return out;
}

std::string PathForMessage(const fs::path& path) {
    return WideToUtf8(path.wstring());
}

bool StartsWithToken(const char* s, char a) {
    return s[0] == a && (s[1] == ' ' || s[1] == '\t');
}

int ResolveObjIndex(long value, size_t count) {
    if (value > 0) return static_cast<int>(value - 1);
    if (value < 0) return static_cast<int>(count) + static_cast<int>(value);
    return -1;
}

bool ParseFaceToken(const std::string& token, size_t positionCount, size_t uvCount, ObjMesh::Corner& out) {
    const char* p = token.c_str();
    char* end = nullptr;
    long vi = std::strtol(p, &end, 10);
    if (end == p) return false;

    int posIndex = ResolveObjIndex(vi, positionCount);
    if (posIndex < 0 || static_cast<size_t>(posIndex) >= positionCount) return false;

    out.position = static_cast<uint32_t>(posIndex);
    out.uv = 0;
    out.hasUV = false;

    p = end;
    if (*p == '/') {
        ++p;
        if (*p != '/' && *p != '\0') {
            long ti = std::strtol(p, &end, 10);
            if (end != p) {
                int uvIndex = ResolveObjIndex(ti, uvCount);
                if (uvIndex >= 0 && static_cast<size_t>(uvIndex) < uvCount) {
                    out.uv = static_cast<uint32_t>(uvIndex);
                    out.hasUV = true;
                }
            }
        }
    }

    return true;
}

bool LoadObjMesh(const fs::path& path,
                 ObjMesh& out,
                 std::atomic<bool>* cancelled,
                 ProgressFn progress,
                 std::string* error)
{
    out = ObjMesh{};
    std::ifstream f(path);
    if (!f.is_open()) {
        if (error) *error = "Failed to open OBJ: " + PathForMessage(path);
        return false;
    }

    f.seekg(0, std::ios::end);
    const std::streampos totalBytes = f.tellg();
    f.seekg(0, std::ios::beg);

    Report(progress, 0.0f, "Reading OBJ");

    std::string line;
    line.reserve(256);
    size_t lineNo = 0;
    while (std::getline(f, line)) {
        ++lineNo;
        if ((lineNo & 0x3FFFu) == 0u) {
            if (Cancelled(cancelled)) {
                if (error) *error = "Cancelled";
                return false;
            }
            if (totalBytes > 0) {
                const double pos = static_cast<double>(f.tellg());
                Report(progress, static_cast<float>(0.45 * pos / static_cast<double>(totalBytes)), "Reading OBJ");
            }
        }

        const char* s = line.c_str();
        while (*s == ' ' || *s == '\t') ++s;
        if (*s == '\0' || *s == '#') continue;

        if (StartsWithToken(s, 'v')) {
            float x = 0.0f, y = 0.0f, z = 0.0f;
            if (std::sscanf(s + 2, "%f %f %f", &x, &y, &z) == 3) {
                out.positions.emplace_back(x, y, z);
            }
            continue;
        }

        if (s[0] == 'v' && s[1] == 't' && (s[2] == ' ' || s[2] == '\t')) {
            float u = 0.0f, v = 0.0f;
            if (std::sscanf(s + 3, "%f %f", &u, &v) >= 2) {
                out.uvs.emplace_back(u, v);
            }
            continue;
        }

        if (StartsWithToken(s, 'f')) {
            std::istringstream iss(s + 2);
            std::vector<ObjMesh::Corner> face;
            std::string token;
            while (iss >> token) {
                ObjMesh::Corner c;
                if (ParseFaceToken(token, out.positions.size(), out.uvs.size(), c)) {
                    face.push_back(c);
                }
            }

            for (size_t i = 1; i + 1 < face.size(); ++i) {
                ObjMesh::Triangle t;
                t.c[0] = face[0];
                t.c[1] = face[i];
                t.c[2] = face[i + 1];
                out.triangles.push_back(t);
            }
        }
    }

    if (out.positions.empty()) {
        if (error) *error = "OBJ contains no vertices";
        return false;
    }
    if (out.triangles.empty()) {
        if (error) *error = "OBJ contains no triangles";
        return false;
    }

    Report(progress, 0.5f, "OBJ loaded");
    return true;
}

Aabb ComputeBounds(const ObjMesh& mesh) {
    Aabb b;
    for (const Vec3& p : mesh.positions) b.Expand(p);
    if (mesh.positions.empty()) {
        b.min = Vec3(0.0f);
        b.max = Vec3(1.0f);
    }
    return b;
}

void ApplyAxisTransform(ObjMesh& mesh, AxisTransform transform) {
    switch (transform) {
        case AxisTransform::SwapYZ:
            for (Vec3& p : mesh.positions) std::swap(p.y, p.z);
            break;
        case AxisTransform::BlenderObjDefault:
            for (Vec3& p : mesh.positions) {
                p = Vec3(p.x, -p.z, p.y);
            }
            break;
        case AxisTransform::None:
        default:
            break;
    }
}

Vec2 PlanarUV(const Vec3& p, const Aabb& b, BasePlane plane) {
    const Vec3 size = Max(b.max - b.min, Vec3(1.0e-6f));
    switch (plane) {
        case BasePlane::XY:
            return Vec2((p.x - b.min.x) / size.x, (p.y - b.min.y) / size.y);
        case BasePlane::ZY:
            return Vec2((p.z - b.min.z) / size.z, (p.y - b.min.y) / size.y);
        case BasePlane::XZ:
        default:
            return Vec2((p.x - b.min.x) / size.x, (p.z - b.min.z) / size.z);
    }
}

Vec3 PlanePoint(float u, float v, const Aabb& b, BasePlane plane, float axisValue) {
    const Vec3 size = b.max - b.min;
    switch (plane) {
        case BasePlane::XY:
            return Vec3(b.min.x + u * size.x, b.min.y + v * size.y, axisValue);
        case BasePlane::ZY:
            return Vec3(axisValue, b.min.y + v * size.y, b.min.z + u * size.z);
        case BasePlane::XZ:
        default:
            return Vec3(b.min.x + u * size.x, axisValue, b.min.z + v * size.z);
    }
}

Vec3 ComputeExactUvValue(const Vec3& objectPosition,
                         const Vec2& uv,
                         const Aabb& bounds,
                         BasePlane plane,
                         const BakeSettings& settings)
{
    Vec3 v = settings.vectorMode == VectorMode::ObjectPosition
        ? objectPosition
        : objectPosition - PlanePoint(uv.x, uv.y, bounds, plane, settings.baseAxisValue);
    return v * settings.outputScale;
}

float Edge2D(const Vec2& a, const Vec2& b, const Vec2& c) {
    return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
}

Vec3 Barycentric2D(const Vec2 uv[3], const Vec2& q, float area) {
    const float w0 = Edge2D(uv[1], uv[2], q) / area;
    const float w1 = Edge2D(uv[2], uv[0], q) / area;
    return Vec3(w0, w1, 1.0f - w0 - w1);
}

void SetBaryComponent(Vec3& v, int idx, float value) {
    if (idx == 0) v.x = value;
    else if (idx == 1) v.y = value;
    else v.z = value;
}

Vec3 ClosestBarycentricOnTri2D(const Vec2 uv[3], const Vec2& q, float area) {
    const Vec3 inside = Barycentric2D(uv, q, area);
    if (inside.x >= 0.0f && inside.y >= 0.0f && inside.z >= 0.0f) {
        return inside;
    }

    Vec3 best(1.0f, 0.0f, 0.0f);
    float bestDistSq = std::numeric_limits<float>::max();

    auto testEdge = [&](int a, int b) {
        const Vec2 ab = uv[b] - uv[a];
        const float denom = Dot(ab, ab);
        const float edgeT = denom > 1.0e-20f
            ? std::clamp(Dot(q - uv[a], ab) / denom, 0.0f, 1.0f)
            : 0.0f;
        const Vec2 closest = uv[a] + ab * edgeT;
        const Vec2 d = closest - q;
        const float distSq = Dot(d, d);
        if (distSq < bestDistSq) {
            bestDistSq = distSq;
            best = Vec3(0.0f);
            SetBaryComponent(best, a, 1.0f - edgeT);
            SetBaryComponent(best, b, edgeT);
        }
    };

    testEdge(0, 1);
    testEdge(1, 2);
    testEdge(2, 0);
    return best;
}

ChannelStats ComputeChannelStats(const std::vector<VDMPixel>& pixels) {
    ChannelStats stats;
    for (const VDMPixel& p : pixels) {
        if (p.a <= 0.0f) continue;
        Vec3 v(p.r, p.g, p.b);
        stats.min = Min(stats.min, v);
        stats.max = Max(stats.max, v);
        ++stats.samples;
    }
    if (stats.samples == 0) {
        stats.min = Vec3(0.0f);
        stats.max = Vec3(0.0f);
    }
    return stats;
}

float DistanceSqToAabb(const Aabb& b, const Vec3& p) {
    float d = 0.0f;
    auto axis = [&](float v, float lo, float hi) {
        if (v < lo) {
            const float e = lo - v;
            d += e * e;
        } else if (v > hi) {
            const float e = v - hi;
            d += e * e;
        }
    };
    axis(p.x, b.min.x, b.max.x);
    axis(p.y, b.min.y, b.max.y);
    axis(p.z, b.min.z, b.max.z);
    return d;
}

Vec3 ClosestPointOnTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ap = p - a;
    const float d1 = Dot(ab, ap);
    const float d2 = Dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;

    const Vec3 bp = p - b;
    const float d3 = Dot(ab, bp);
    const float d4 = Dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        const float v = d1 / (d1 - d3);
        return a + ab * v;
    }

    const Vec3 cp = p - c;
    const float d5 = Dot(ab, cp);
    const float d6 = Dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;

    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        const float w = d2 / (d2 - d6);
        return a + ac * w;
    }

    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }

    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;
    return a + ab * v + ac * w;
}

class TriangleBVH {
public:
    struct Face {
        uint32_t a = 0;
        uint32_t b = 0;
        uint32_t c = 0;
    };

    bool Build(std::vector<Vec3> vertices, std::vector<Face> faces) {
        vertices_ = std::move(vertices);
        faces_ = std::move(faces);
        indices_.resize(faces_.size());
        triBounds_.resize(faces_.size());
        triCentroids_.resize(faces_.size());
        for (size_t i = 0; i < faces_.size(); ++i) {
            indices_[i] = static_cast<int>(i);
            const Face& f = faces_[i];
            Aabb b;
            b.Expand(vertices_[f.a]);
            b.Expand(vertices_[f.b]);
            b.Expand(vertices_[f.c]);
            triBounds_[i] = b;
            triCentroids_[i] = (vertices_[f.a] + vertices_[f.b] + vertices_[f.c]) / 3.0f;
        }
        nodes_.clear();
        if (faces_.empty()) return false;
        BuildNode(0, static_cast<int>(indices_.size()));
        return !nodes_.empty();
    }

    bool Empty() const { return nodes_.empty(); }

    Vec3 ClosestPoint(const Vec3& p) const {
        float bestDistSq = std::numeric_limits<float>::max();
        Vec3 best = p;
        if (nodes_.empty()) return best;

        std::vector<int> stack;
        stack.reserve(64);
        stack.push_back(0);
        while (!stack.empty()) {
            const int ni = stack.back();
            stack.pop_back();
            const Node& n = nodes_[ni];
            if (DistanceSqToAabb(n.box, p) > bestDistSq) continue;

            if (n.count > 0) {
                for (int i = 0; i < n.count; ++i) {
                    const int triIndex = indices_[n.start + i];
                    const Face& f = faces_[triIndex];
                    Vec3 q = ClosestPointOnTriangle(p, vertices_[f.a], vertices_[f.b], vertices_[f.c]);
                    const float dsq = Dot(q - p, q - p);
                    if (dsq < bestDistSq) {
                        bestDistSq = dsq;
                        best = q;
                    }
                }
            } else {
                const float dl = DistanceSqToAabb(nodes_[n.left].box, p);
                const float dr = DistanceSqToAabb(nodes_[n.right].box, p);
                if (dl < dr) {
                    if (dr <= bestDistSq) stack.push_back(n.right);
                    if (dl <= bestDistSq) stack.push_back(n.left);
                } else {
                    if (dl <= bestDistSq) stack.push_back(n.left);
                    if (dr <= bestDistSq) stack.push_back(n.right);
                }
            }
        }
        return best;
    }

private:
    struct Node {
        Aabb box;
        int left = -1;
        int right = -1;
        int start = 0;
        int count = 0;
    };

    int BuildNode(int start, int count) {
        Node n;
        for (int i = 0; i < count; ++i) {
            n.box.Expand(triBounds_[indices_[start + i]]);
        }

        const int nodeIndex = static_cast<int>(nodes_.size());
        nodes_.push_back(n);

        if (count <= 8) {
            nodes_[nodeIndex].start = start;
            nodes_[nodeIndex].count = count;
            return nodeIndex;
        }

        Aabb centroidBox;
        for (int i = 0; i < count; ++i) {
            centroidBox.Expand(triCentroids_[indices_[start + i]]);
        }
        const Vec3 extent = centroidBox.max - centroidBox.min;
        int axis = 0;
        if (extent.y > extent.x && extent.y >= extent.z) axis = 1;
        else if (extent.z > extent.x && extent.z >= extent.y) axis = 2;

        const int mid = start + count / 2;
        auto coord = [&](int tri) {
            const Vec3& c = triCentroids_[tri];
            return axis == 0 ? c.x : (axis == 1 ? c.y : c.z);
        };
        std::nth_element(indices_.begin() + start,
                         indices_.begin() + mid,
                         indices_.begin() + start + count,
                         [&](int a, int b) { return coord(a) < coord(b); });

        const int left = BuildNode(start, mid - start);
        const int right = BuildNode(mid, start + count - mid);
        nodes_[nodeIndex].left = left;
        nodes_[nodeIndex].right = right;
        nodes_[nodeIndex].count = 0;
        return nodeIndex;
    }

    std::vector<Vec3> vertices_;
    std::vector<Face> faces_;
    std::vector<int> indices_;
    std::vector<Aabb> triBounds_;
    std::vector<Vec3> triCentroids_;
    std::vector<Node> nodes_;
};

bool BuildTargetBVH(const ObjMesh& mesh, TriangleBVH& out, std::string* error) {
    std::vector<TriangleBVH::Face> faces;
    faces.reserve(mesh.triangles.size());
    for (const ObjMesh::Triangle& tri : mesh.triangles) {
        const uint32_t a = tri.c[0].position;
        const uint32_t b = tri.c[1].position;
        const uint32_t c = tri.c[2].position;
        if (a >= mesh.positions.size() || b >= mesh.positions.size() || c >= mesh.positions.size()) continue;
        if (a == b || b == c || c == a) continue;
        const Vec3 ab = mesh.positions[b] - mesh.positions[a];
        const Vec3 ac = mesh.positions[c] - mesh.positions[a];
        if (Dot(Cross(ab, ac), Cross(ab, ac)) <= 1.0e-20f) continue;
        faces.push_back({a, b, c});
    }
    if (faces.empty()) {
        if (error) *error = "Target/detail OBJ has no valid triangles for closest-surface projection";
        return false;
    }
    if (!out.Build(mesh.positions, std::move(faces))) {
        if (error) *error = "Failed to build target/detail mesh BVH";
        return false;
    }
    return true;
}

void PushCString(std::vector<uint8_t>& out, const char* s) {
    while (*s) out.push_back(static_cast<uint8_t>(*s++));
    out.push_back(0);
}

void PushU8(std::vector<uint8_t>& out, uint8_t v) { out.push_back(v); }

void PushU32(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back(static_cast<uint8_t>(v & 0xFFu));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFFu));
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xFFu));
    out.push_back(static_cast<uint8_t>((v >> 24) & 0xFFu));
}

void PushI32(std::vector<uint8_t>& out, int32_t v) {
    PushU32(out, static_cast<uint32_t>(v));
}

void PushU64(std::vector<uint8_t>& out, uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        out.push_back(static_cast<uint8_t>((v >> (i * 8)) & 0xFFu));
    }
}

void PushFloat(std::vector<uint8_t>& out, float v) {
    uint32_t bits = 0;
    static_assert(sizeof(float) == sizeof(uint32_t), "float must be 32-bit");
    std::memcpy(&bits, &v, sizeof(float));
    PushU32(out, bits);
}

void PushAttr(std::vector<uint8_t>& header,
              const char* name,
              const char* type,
              const std::vector<uint8_t>& data)
{
    PushCString(header, name);
    PushCString(header, type);
    PushU32(header, static_cast<uint32_t>(data.size()));
    header.insert(header.end(), data.begin(), data.end());
}

bool SaveEXR32F(const fs::path& path,
                const std::vector<VDMPixel>& pixels,
                int width,
                int height,
                std::string* error)
{
    if (width <= 0 || height <= 0 || pixels.size() != static_cast<size_t>(width) * height) {
        if (error) *error = "Invalid EXR image dimensions";
        return false;
    }

    std::vector<uint8_t> header;

    std::vector<uint8_t> chlist;
    const char* channels[] = {"A", "B", "G", "R"};
    for (const char* ch : channels) {
        PushCString(chlist, ch);
        PushI32(chlist, 2);
        PushU8(chlist, 0);
        PushU8(chlist, 0);
        PushU8(chlist, 0);
        PushU8(chlist, 0);
        PushI32(chlist, 1);
        PushI32(chlist, 1);
    }
    PushU8(chlist, 0);
    PushAttr(header, "channels", "chlist", chlist);

    std::vector<uint8_t> compression;
    PushU8(compression, 0);
    PushAttr(header, "compression", "compression", compression);

    std::vector<uint8_t> box;
    PushI32(box, 0);
    PushI32(box, 0);
    PushI32(box, width - 1);
    PushI32(box, height - 1);
    PushAttr(header, "dataWindow", "box2i", box);
    PushAttr(header, "displayWindow", "box2i", box);

    std::vector<uint8_t> lineOrder;
    PushU8(lineOrder, 0);
    PushAttr(header, "lineOrder", "lineOrder", lineOrder);

    std::vector<uint8_t> oneFloat;
    PushFloat(oneFloat, 1.0f);
    PushAttr(header, "pixelAspectRatio", "float", oneFloat);

    std::vector<uint8_t> center;
    PushFloat(center, 0.0f);
    PushFloat(center, 0.0f);
    PushAttr(header, "screenWindowCenter", "v2f", center);
    PushAttr(header, "screenWindowWidth", "float", oneFloat);

    PushU8(header, 0);

    const uint64_t headerAndMagicBytes = 8 + static_cast<uint64_t>(header.size());
    const uint64_t tableBytes = static_cast<uint64_t>(height) * 8ull;
    const uint64_t scanlineBytes = static_cast<uint64_t>(width) * 4ull * sizeof(float);
    const uint64_t chunkBytes = 8ull + scanlineBytes;
    const uint64_t firstChunkOffset = headerAndMagicBytes + tableBytes;

    std::ofstream f(path, std::ios::binary);
    if (!f.is_open()) {
        if (error) *error = "Failed to open EXR output: " + PathForMessage(path);
        return false;
    }

    std::vector<uint8_t> fileHeader;
    PushU32(fileHeader, 20000630u);
    PushU32(fileHeader, 2u);
    f.write(reinterpret_cast<const char*>(fileHeader.data()), static_cast<std::streamsize>(fileHeader.size()));
    f.write(reinterpret_cast<const char*>(header.data()), static_cast<std::streamsize>(header.size()));

    std::vector<uint8_t> offsets;
    offsets.reserve(static_cast<size_t>(tableBytes));
    for (int y = 0; y < height; ++y) {
        PushU64(offsets, firstChunkOffset + static_cast<uint64_t>(y) * chunkBytes);
    }
    f.write(reinterpret_cast<const char*>(offsets.data()), static_cast<std::streamsize>(offsets.size()));

    std::vector<uint8_t> row;
    row.reserve(static_cast<size_t>(8 + scanlineBytes));
    for (int y = 0; y < height; ++y) {
        row.clear();
        PushI32(row, y);
        PushU32(row, static_cast<uint32_t>(scanlineBytes));

        for (const char* ch : channels) {
            for (int x = 0; x < width; ++x) {
                const VDMPixel& p = pixels[static_cast<size_t>(y) * width + x];
                float v = 0.0f;
                switch (ch[0]) {
                    case 'A': v = p.a; break;
                    case 'B': v = p.b; break;
                    case 'G': v = p.g; break;
                    case 'R': v = p.r; break;
                    default: break;
                }
                PushFloat(row, v);
            }
        }
        f.write(reinterpret_cast<const char*>(row.data()), static_cast<std::streamsize>(row.size()));
    }

    if (!f.good()) {
        if (error) *error = "Failed while writing EXR data";
        return false;
    }
    return true;
}

bool BakeSingleExactUvToPixels(const ObjMesh& mesh,
                               const BakeSettings& settings,
                               std::vector<VDMPixel>& outPixels,
                               BakeResult& result,
                               std::atomic<bool>* cancelled,
                               ProgressFn progress)
{
    result = {};
    result.width = settings.width;
    result.height = settings.height;
    result.triangleCount = mesh.triangles.size();

    if (settings.width <= 0 || settings.height <= 0) {
        result.message = "Invalid output resolution";
        return false;
    }
    if (!mesh.HasAnyUVs()) {
        result.message = "Exact UV XYZ bake requires OBJ UVs referenced by faces (f v/vt[/vn])";
        return false;
    }

    const Aabb bounds = ComputeBounds(mesh);
    Vec2 uvMin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2 uvMax(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
    if (settings.normalizeObjUVs) {
        for (const Vec2& uv : mesh.uvs) {
            uvMin = Min(uvMin, uv);
            uvMax = Max(uvMax, uv);
        }
    }
    const Vec2 uvSpan = Max(uvMax - uvMin, Vec2(1.0e-6f, 1.0e-6f));

    std::vector<PreparedTri> tris;
    tris.reserve(mesh.triangles.size());
    size_t skippedMissingUVs = 0;
    size_t objUvTriangles = 0;

    Report(progress, 0.52f, "Preparing exact UV XYZ triangles");
    for (size_t i = 0; i < mesh.triangles.size(); ++i) {
        if (Cancelled(cancelled)) {
            result.message = "Cancelled";
            return false;
        }

        const ObjMesh::Triangle& src = mesh.triangles[i];
        PreparedTri t;
        bool valid = true;
        for (int k = 0; k < 3; ++k) {
            if (src.c[k].position >= mesh.positions.size() ||
                !src.c[k].hasUV ||
                src.c[k].uv >= mesh.uvs.size()) {
                valid = false;
                break;
            }
            t.p[k] = mesh.positions[src.c[k].position];
            t.uv[k] = mesh.uvs[src.c[k].uv];
            if (settings.normalizeObjUVs) {
                t.uv[k].x = (t.uv[k].x - uvMin.x) / uvSpan.x;
                t.uv[k].y = (t.uv[k].y - uvMin.y) / uvSpan.y;
            }
            if (settings.flipV) {
                t.uv[k].y = 1.0f - t.uv[k].y;
            }
        }
        if (!valid) {
            ++skippedMissingUVs;
            continue;
        }
        ++objUvTriangles;
        tris.push_back(t);

        if ((i & 0x3FFFu) == 0u && !mesh.triangles.empty()) {
            const float pr = 0.52f + 0.08f * static_cast<float>(i) / static_cast<float>(mesh.triangles.size());
            Report(progress, pr, "Preparing exact UV XYZ triangles");
        }
    }

    if (tris.empty()) {
        result.message = "Exact UV XYZ bake found no triangles with complete OBJ UVs";
        return false;
    }

    const int width = settings.width;
    const int height = settings.height;
    outPixels.assign(static_cast<size_t>(width) * height, VDMPixel{});

    Report(progress, 0.60f, "Rasterizing VDM");
    size_t covered = 0;
    size_t padded = 0;
    constexpr float eps = -1.0e-5f;

    for (size_t ti = 0; ti < tris.size(); ++ti) {
        if (Cancelled(cancelled)) {
            result.message = "Cancelled";
            return false;
        }

        const PreparedTri& t = tris[ti];
        const float area = Edge2D(t.uv[0], t.uv[1], t.uv[2]);
        if (std::abs(area) < 1.0e-12f) continue;

        const float minU = std::min({t.uv[0].x, t.uv[1].x, t.uv[2].x});
        const float maxU = std::max({t.uv[0].x, t.uv[1].x, t.uv[2].x});
        const float minV = std::min({t.uv[0].y, t.uv[1].y, t.uv[2].y});
        const float maxV = std::max({t.uv[0].y, t.uv[1].y, t.uv[2].y});
        if (maxU < 0.0f || minU > 1.0f || maxV < 0.0f || minV > 1.0f) continue;

        const int x0 = std::max(0, static_cast<int>(std::floor(minU * width)));
        const int x1 = std::min(width - 1, static_cast<int>(std::ceil(maxU * width)));
        const int y0 = std::max(0, static_cast<int>(std::floor(minV * height)));
        const int y1 = std::min(height - 1, static_cast<int>(std::ceil(maxV * height)));

        const float invArea = 1.0f / area;
        const float e0dx = t.uv[2].x - t.uv[1].x;
        const float e0dy = t.uv[2].y - t.uv[1].y;
        const float e0c = t.uv[1].y * e0dx - t.uv[1].x * e0dy;
        const float e1dx = t.uv[0].x - t.uv[2].x;
        const float e1dy = t.uv[0].y - t.uv[2].y;
        const float e1c = t.uv[2].y * e1dx - t.uv[2].x * e1dy;

        for (int y = y0; y <= y1; ++y) {
            const float screenV = (static_cast<float>(y) + 0.5f) / static_cast<float>(height);
            for (int x = x0; x <= x1; ++x) {
                const float screenU = (static_cast<float>(x) + 0.5f) / static_cast<float>(width);
                const float w0 = (screenU * e0dy - screenV * e0dx + e0c) * invArea;
                const float w1 = (screenU * e1dy - screenV * e1dx + e1c) * invArea;
                const float w2 = 1.0f - w0 - w1;
                if (w0 < eps || w1 < eps || w2 < eps) continue;

                const Vec3 p = t.p[0] * w0 + t.p[1] * w1 + t.p[2] * w2;
                const Vec2 uvAtPoint = t.uv[0] * w0 + t.uv[1] * w1 + t.uv[2] * w2;
                const Vec3 v = ComputeExactUvValue(p, uvAtPoint, bounds, settings.basePlane, settings);

                VDMPixel& dst = outPixels[static_cast<size_t>(y) * width + x];
                if (dst.a <= 0.0f) ++covered;
                dst.r = v.x;
                dst.g = v.y;
                dst.b = v.z;
                dst.a = 1.0f;
            }
        }

        if ((ti & 0x7FFu) == 0u && !tris.empty()) {
            const float pr = 0.60f + 0.32f * static_cast<float>(ti) / static_cast<float>(tris.size());
            Report(progress, pr, "Rasterizing VDM");
        }
    }

    const int edgePadding = std::max(0, settings.edgePadding);
    if (edgePadding > 0 && covered >= static_cast<size_t>(width) * height) {
        Report(progress, 0.93f, "Skipping edge extrapolation; all pixels are covered");
    } else if (edgePadding > 0 && covered > 0) {
        Report(progress, 0.93f, "Extrapolating mesh VDM edges");
        const float padU = static_cast<float>(edgePadding + 1) / static_cast<float>(std::max(1, width));
        const float padV = static_cast<float>(edgePadding + 1) / static_cast<float>(std::max(1, height));
        const float maxDistPxSq = static_cast<float>(edgePadding * edgePadding);
        std::vector<float> bestDistSq(outPixels.size(), std::numeric_limits<float>::max());

        for (size_t ti = 0; ti < tris.size(); ++ti) {
            if (Cancelled(cancelled)) {
                result.message = "Cancelled";
                return false;
            }

            const PreparedTri& t = tris[ti];
            const float area = Edge2D(t.uv[0], t.uv[1], t.uv[2]);
            if (std::abs(area) < 1.0e-12f) continue;

            const float minU = std::min({t.uv[0].x, t.uv[1].x, t.uv[2].x}) - padU;
            const float maxU = std::max({t.uv[0].x, t.uv[1].x, t.uv[2].x}) + padU;
            const float minV = std::min({t.uv[0].y, t.uv[1].y, t.uv[2].y}) - padV;
            const float maxV = std::max({t.uv[0].y, t.uv[1].y, t.uv[2].y}) + padV;
            if (maxU < 0.0f || minU > 1.0f || maxV < 0.0f || minV > 1.0f) continue;

            const int x0 = std::max(0, static_cast<int>(std::floor(minU * width)));
            const int x1 = std::min(width - 1, static_cast<int>(std::ceil(maxU * width)));
            const int y0 = std::max(0, static_cast<int>(std::floor(minV * height)));
            const int y1 = std::min(height - 1, static_cast<int>(std::ceil(maxV * height)));

            for (int y = y0; y <= y1; ++y) {
                const float screenV = (static_cast<float>(y) + 0.5f) / static_cast<float>(height);
                for (int x = x0; x <= x1; ++x) {
                    const size_t idx = static_cast<size_t>(y) * width + x;
                    if (outPixels[idx].a > 0.0f) continue;

                    const float screenU = (static_cast<float>(x) + 0.5f) / static_cast<float>(width);
                    const Vec2 q(screenU, screenV);
                    const Vec3 closestBary = ClosestBarycentricOnTri2D(t.uv, q, area);
                    const Vec2 closestUv = t.uv[0] * closestBary.x + t.uv[1] * closestBary.y + t.uv[2] * closestBary.z;
                    const float dxPx = (closestUv.x - q.x) * static_cast<float>(width);
                    const float dyPx = (closestUv.y - q.y) * static_cast<float>(height);
                    const float distPxSq = dxPx * dxPx + dyPx * dyPx;
                    if (distPxSq > maxDistPxSq || distPxSq >= bestDistSq[idx]) continue;

                    const Vec3 bary = Barycentric2D(t.uv, q, area);
                    const Vec3 p = t.p[0] * bary.x + t.p[1] * bary.y + t.p[2] * bary.z;
                    const Vec3 v = ComputeExactUvValue(p, q, bounds, settings.basePlane, settings);

                    if (bestDistSq[idx] == std::numeric_limits<float>::max()) ++padded;
                    bestDistSq[idx] = distPxSq;
                    VDMPixel& dst = outPixels[idx];
                    dst.r = v.x;
                    dst.g = v.y;
                    dst.b = v.z;
                    dst.a = 0.0f;
                }
            }

            if ((ti & 0x7FFu) == 0u && !tris.empty()) {
                const float pr = 0.93f + 0.04f * static_cast<float>(ti) / static_cast<float>(tris.size());
                Report(progress, pr, "Extrapolating mesh VDM edges");
            }
        }

        for (size_t i = 0; i < bestDistSq.size(); ++i) {
            if (bestDistSq[i] != std::numeric_limits<float>::max()) {
                outPixels[i].a = 1.0f;
            }
        }
    }

    result.success = true;
    result.coveredPixels = covered;
    const ChannelStats stats = ComputeChannelStats(outPixels);
    std::ostringstream ss;
    ss << "Baked exact UV XYZ "
       << (settings.vectorMode == VectorMode::ObjectPosition ? "object-position" : "displacement")
       << " VDM from OBJ UVs"
       << "; UV tris obj/planar=" << objUvTriangles << "/0"
       << "; RGB min=(" << stats.min.x << "," << stats.min.y << "," << stats.min.z << ")"
       << " max=(" << stats.max.x << "," << stats.max.y << "," << stats.max.z << ")"
       << "; extrapolated " << padded << " edge-padding pixels";
    if (skippedMissingUVs > 0) {
        ss << "; skipped " << skippedMissingUVs << " triangles without complete UVs";
    }
    result.message = ss.str();
    Report(progress, 0.98f, "VDM pixels ready");
    return true;
}

bool BakeTwoMeshToPixels(const ObjMesh& restMesh,
                         const ObjMesh& targetMesh,
                         const BakeSettings& settings,
                         std::vector<VDMPixel>& outPixels,
                         BakeResult& result,
                         std::atomic<bool>* cancelled,
                         ProgressFn progress)
{
    result = {};
    result.width = settings.width;
    result.height = settings.height;
    result.triangleCount = restMesh.triangles.size();

    if (settings.width <= 0 || settings.height <= 0) {
        result.message = "Invalid output resolution";
        return false;
    }
    if (restMesh.positions.empty() || restMesh.triangles.empty()) {
        result.message = "Rest/base OBJ contains no mesh triangles";
        return false;
    }
    if (targetMesh.positions.empty()) {
        result.message = "Target/detail OBJ contains no vertices";
        return false;
    }

    const bool matchedVertexOrder =
        restMesh.positions.size() == targetMesh.positions.size() &&
        restMesh.triangles.size() == targetMesh.triangles.size();

    TriangleBVH targetBvh;
    size_t targetTriCount = targetMesh.triangles.size();
    if (!matchedVertexOrder) {
        Report(progress, 0.50f, "Building target mesh BVH");
        std::string err;
        if (!BuildTargetBVH(targetMesh, targetBvh, &err)) {
            result.message = err;
            return false;
        }
    }

    const Aabb restBounds = ComputeBounds(restMesh);
    const bool objUvAvailable = restMesh.HasAnyUVs();
    Vec2 uvMin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2 uvMax(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
    if (objUvAvailable && settings.normalizeObjUVs) {
        for (const Vec2& uv : restMesh.uvs) {
            uvMin = Min(uvMin, uv);
            uvMax = Max(uvMax, uv);
        }
    }
    const Vec2 uvSpan = Max(uvMax - uvMin, Vec2(1.0e-6f, 1.0e-6f));

    std::vector<TwoMeshPreparedTri> tris;
    tris.reserve(restMesh.triangles.size());
    size_t objUvTriangles = 0;
    size_t planarUvTriangles = 0;
    size_t skippedInvalidRest = 0;

    Report(progress, 0.54f, "Preparing rest/domain triangles");
    for (size_t i = 0; i < restMesh.triangles.size(); ++i) {
        if (Cancelled(cancelled)) {
            result.message = "Cancelled";
            return false;
        }

        const ObjMesh::Triangle& src = restMesh.triangles[i];
        TwoMeshPreparedTri t;
        bool validRestPositions = true;
        bool triHasAllObjUVs = objUvAvailable;
        for (int k = 0; k < 3; ++k) {
            if (src.c[k].position >= restMesh.positions.size()) {
                validRestPositions = false;
                break;
            }
            t.restP[k] = restMesh.positions[src.c[k].position];
            if (matchedVertexOrder) {
                t.targetP[k] = targetMesh.positions[src.c[k].position];
            }
            if (!src.c[k].hasUV || src.c[k].uv >= restMesh.uvs.size()) {
                triHasAllObjUVs = false;
            }
        }
        if (!validRestPositions) {
            ++skippedInvalidRest;
            continue;
        }

        if (triHasAllObjUVs) ++objUvTriangles;
        else ++planarUvTriangles;

        for (int k = 0; k < 3; ++k) {
            if (triHasAllObjUVs) {
                const Vec2 sourceUv = restMesh.uvs[src.c[k].uv];
                t.uv[k] = sourceUv;
                if (settings.normalizeObjUVs) {
                    t.uv[k].x = (sourceUv.x - uvMin.x) / uvSpan.x;
                    t.uv[k].y = (sourceUv.y - uvMin.y) / uvSpan.y;
                }
            } else {
                t.uv[k] = PlanarUV(t.restP[k], restBounds, settings.basePlane);
            }
            if (settings.flipV) {
                t.uv[k].y = 1.0f - t.uv[k].y;
            }
        }

        tris.push_back(t);
        if ((i & 0x3FFFu) == 0u && !restMesh.triangles.empty()) {
            const float pr = 0.54f + 0.06f * static_cast<float>(i) / static_cast<float>(restMesh.triangles.size());
            Report(progress, pr, "Preparing rest/domain triangles");
        }
    }

    if (tris.empty()) {
        result.message = "No valid rest/base domain triangles to bake";
        return false;
    }

    const int width = settings.width;
    const int height = settings.height;
    outPixels.assign(static_cast<size_t>(width) * height, VDMPixel{});

    auto sampleTarget = [&](const TwoMeshPreparedTri& t, const Vec3& bary, const Vec3& restP) {
        if (matchedVertexOrder) {
            return t.targetP[0] * bary.x + t.targetP[1] * bary.y + t.targetP[2] * bary.z;
        }
        return targetBvh.ClosestPoint(restP);
    };

    auto encode = [&](const Vec3& restP, const Vec3& targetP) {
        Vec3 v = settings.vectorMode == VectorMode::ObjectPosition ? targetP : targetP - restP;
        return v * settings.outputScale;
    };

    Report(progress, 0.62f, "Rasterizing two-mesh VDM");
    size_t covered = 0;
    size_t padded = 0;
    constexpr float eps = -1.0e-5f;

    for (size_t ti = 0; ti < tris.size(); ++ti) {
        if (Cancelled(cancelled)) {
            result.message = "Cancelled";
            return false;
        }

        const TwoMeshPreparedTri& t = tris[ti];
        const float area = Edge2D(t.uv[0], t.uv[1], t.uv[2]);
        if (std::abs(area) < 1.0e-12f) continue;

        const float minU = std::min({t.uv[0].x, t.uv[1].x, t.uv[2].x});
        const float maxU = std::max({t.uv[0].x, t.uv[1].x, t.uv[2].x});
        const float minV = std::min({t.uv[0].y, t.uv[1].y, t.uv[2].y});
        const float maxV = std::max({t.uv[0].y, t.uv[1].y, t.uv[2].y});
        if (maxU < 0.0f || minU > 1.0f || maxV < 0.0f || minV > 1.0f) continue;

        const int x0 = std::max(0, static_cast<int>(std::floor(minU * width)));
        const int x1 = std::min(width - 1, static_cast<int>(std::ceil(maxU * width)));
        const int y0 = std::max(0, static_cast<int>(std::floor(minV * height)));
        const int y1 = std::min(height - 1, static_cast<int>(std::ceil(maxV * height)));

        const float invArea = 1.0f / area;
        const float e0dx = t.uv[2].x - t.uv[1].x;
        const float e0dy = t.uv[2].y - t.uv[1].y;
        const float e0c = t.uv[1].y * e0dx - t.uv[1].x * e0dy;
        const float e1dx = t.uv[0].x - t.uv[2].x;
        const float e1dy = t.uv[0].y - t.uv[2].y;
        const float e1c = t.uv[2].y * e1dx - t.uv[2].x * e1dy;

        for (int y = y0; y <= y1; ++y) {
            const float screenV = (static_cast<float>(y) + 0.5f) / static_cast<float>(height);
            for (int x = x0; x <= x1; ++x) {
                const float screenU = (static_cast<float>(x) + 0.5f) / static_cast<float>(width);
                const float w0 = (screenU * e0dy - screenV * e0dx + e0c) * invArea;
                const float w1 = (screenU * e1dy - screenV * e1dx + e1c) * invArea;
                const float w2 = 1.0f - w0 - w1;
                if (w0 < eps || w1 < eps || w2 < eps) continue;

                const Vec3 bary(w0, w1, w2);
                const Vec3 restP = t.restP[0] * bary.x + t.restP[1] * bary.y + t.restP[2] * bary.z;
                const Vec3 targetP = sampleTarget(t, bary, restP);
                const Vec3 v = encode(restP, targetP);

                VDMPixel& dst = outPixels[static_cast<size_t>(y) * width + x];
                if (dst.a <= 0.0f) ++covered;
                dst.r = v.x;
                dst.g = v.y;
                dst.b = v.z;
                dst.a = 1.0f;
            }
        }

        if ((ti & 0x7FFu) == 0u && !tris.empty()) {
            const float pr = 0.62f + 0.28f * static_cast<float>(ti) / static_cast<float>(tris.size());
            Report(progress, pr, "Rasterizing two-mesh VDM");
        }
    }

    const int edgePadding = std::max(0, settings.edgePadding);
    if (edgePadding > 0 && covered >= static_cast<size_t>(width) * height) {
        Report(progress, 0.92f, "Skipping edge extrapolation; all pixels are covered");
    } else if (edgePadding > 0 && covered > 0) {
        Report(progress, 0.92f, "Extrapolating two-mesh VDM edges");
        const float padU = static_cast<float>(edgePadding + 1) / static_cast<float>(std::max(1, width));
        const float padV = static_cast<float>(edgePadding + 1) / static_cast<float>(std::max(1, height));
        const float maxDistPxSq = static_cast<float>(edgePadding * edgePadding);
        std::vector<float> bestDistSq(outPixels.size(), std::numeric_limits<float>::max());

        for (size_t ti = 0; ti < tris.size(); ++ti) {
            if (Cancelled(cancelled)) {
                result.message = "Cancelled";
                return false;
            }

            const TwoMeshPreparedTri& t = tris[ti];
            const float area = Edge2D(t.uv[0], t.uv[1], t.uv[2]);
            if (std::abs(area) < 1.0e-12f) continue;

            const float minU = std::min({t.uv[0].x, t.uv[1].x, t.uv[2].x}) - padU;
            const float maxU = std::max({t.uv[0].x, t.uv[1].x, t.uv[2].x}) + padU;
            const float minV = std::min({t.uv[0].y, t.uv[1].y, t.uv[2].y}) - padV;
            const float maxV = std::max({t.uv[0].y, t.uv[1].y, t.uv[2].y}) + padV;
            if (maxU < 0.0f || minU > 1.0f || maxV < 0.0f || minV > 1.0f) continue;

            const int x0 = std::max(0, static_cast<int>(std::floor(minU * width)));
            const int x1 = std::min(width - 1, static_cast<int>(std::ceil(maxU * width)));
            const int y0 = std::max(0, static_cast<int>(std::floor(minV * height)));
            const int y1 = std::min(height - 1, static_cast<int>(std::ceil(maxV * height)));

            Vec2 uv[3] = {t.uv[0], t.uv[1], t.uv[2]};
            for (int y = y0; y <= y1; ++y) {
                const float screenV = (static_cast<float>(y) + 0.5f) / static_cast<float>(height);
                for (int x = x0; x <= x1; ++x) {
                    const size_t idx = static_cast<size_t>(y) * width + x;
                    if (outPixels[idx].a > 0.0f) continue;

                    const float screenU = (static_cast<float>(x) + 0.5f) / static_cast<float>(width);
                    const Vec2 q(screenU, screenV);
                    const Vec3 closestBary = ClosestBarycentricOnTri2D(uv, q, area);
                    const Vec2 closestUv = uv[0] * closestBary.x + uv[1] * closestBary.y + uv[2] * closestBary.z;
                    const float dxPx = (closestUv.x - q.x) * static_cast<float>(width);
                    const float dyPx = (closestUv.y - q.y) * static_cast<float>(height);
                    const float distPxSq = dxPx * dxPx + dyPx * dyPx;
                    if (distPxSq > maxDistPxSq || distPxSq >= bestDistSq[idx]) continue;

                    const Vec3 bary = Barycentric2D(uv, q, area);
                    const Vec3 restP = t.restP[0] * bary.x + t.restP[1] * bary.y + t.restP[2] * bary.z;
                    const Vec3 targetP = sampleTarget(t, bary, restP);
                    const Vec3 v = encode(restP, targetP);

                    if (bestDistSq[idx] == std::numeric_limits<float>::max()) ++padded;
                    bestDistSq[idx] = distPxSq;
                    VDMPixel& dst = outPixels[idx];
                    dst.r = v.x;
                    dst.g = v.y;
                    dst.b = v.z;
                    dst.a = 0.0f;
                }
            }

            if ((ti & 0x7FFu) == 0u && !tris.empty()) {
                const float pr = 0.92f + 0.05f * static_cast<float>(ti) / static_cast<float>(tris.size());
                Report(progress, pr, "Extrapolating two-mesh VDM edges");
            }
        }

        for (size_t i = 0; i < bestDistSq.size(); ++i) {
            if (bestDistSq[i] != std::numeric_limits<float>::max()) {
                outPixels[i].a = 1.0f;
            }
        }
    }

    result.success = true;
    result.coveredPixels = covered;
    const ChannelStats stats = ComputeChannelStats(outPixels);
    std::ostringstream ss;
    ss << "Baked two-mesh rest->target "
       << (settings.vectorMode == VectorMode::ObjectPosition ? "object-position" : "displacement")
       << " VDM"
       << "; rest/domain UV tris obj/planar=" << objUvTriangles << "/" << planarUvTriangles
       << "; target sample=" << (matchedVertexOrder ? "matched vertex order" : "closest surface")
       << "; target tris=" << targetTriCount
       << "; RGB min=(" << stats.min.x << "," << stats.min.y << "," << stats.min.z << ")"
       << " max=(" << stats.max.x << "," << stats.max.y << "," << stats.max.z << ")"
       << "; extrapolated " << padded << " edge-padding pixels";
    if (skippedInvalidRest > 0) {
        ss << "; skipped " << skippedInvalidRest << " invalid rest triangles";
    }
    if (planarUvTriangles > 0) {
        ss << "; note: generated planar rest domain was used for triangles without complete rest OBJ UVs";
    }
    if (!matchedVertexOrder) {
        ss << "; note: closest-surface mode projects each rest sample to nearest target surface";
    }
    result.message = ss.str();
    Report(progress, 0.98f, "VDM pixels ready");
    return true;
}

bool BakeFromSingleOBJ(const fs::path& inputPath,
                       const fs::path& outputPath,
                       const BakeSettings& settings,
                       BakeResult& result,
                       std::atomic<bool>* cancelled,
                       ProgressFn progress)
{
    ObjMesh mesh;
    std::string err;
    if (!LoadObjMesh(inputPath, mesh, cancelled, progress, &err)) {
        result = {};
        result.message = err.empty() ? "Failed to load OBJ" : err;
        return false;
    }
    ApplyAxisTransform(mesh, settings.axisTransform);

    std::vector<VDMPixel> pixels;
    if (!BakeSingleExactUvToPixels(mesh, settings, pixels, result, cancelled, progress)) return false;
    const std::string bakeSummary = result.message;

    if (Cancelled(cancelled)) {
        result.success = false;
        result.message = "Cancelled";
        return false;
    }

    Report(progress, 0.99f, "Writing EXR");
    if (!SaveEXR32F(outputPath, pixels, settings.width, settings.height, &err)) {
        result.success = false;
        result.message = err.empty() ? "Failed to write EXR output" : err;
        return false;
    }

    result.success = true;
    std::ostringstream ss;
    ss << "Wrote " << PathForMessage(outputPath) << " (" << settings.width << "x" << settings.height
       << ", covered " << result.coveredPixels << " pixels)";
    if (!bakeSummary.empty()) ss << "; " << bakeSummary;
    result.message = ss.str();
    Report(progress, 1.0f, "Done");
    return true;
}

bool BakeFromOBJPair(const fs::path& restPath,
                     const fs::path& targetPath,
                     const fs::path& outputPath,
                     const BakeSettings& settings,
                     BakeResult& result,
                     std::atomic<bool>* cancelled,
                     ProgressFn progress)
{
    auto loadProgress = [&](const char* label, float start, float span) {
        return [&, label, start, span](float p, const std::string& s) {
            std::string text = s;
            if (s == "Reading OBJ") text = std::string("Reading ") + label + " OBJ";
            else if (s == "OBJ loaded") text = std::string(label) + " OBJ loaded";
            Report(progress, start + span * p, text);
        };
    };

    ObjMesh restMesh;
    ObjMesh targetMesh;
    std::string err;

    auto restProgress = loadProgress("rest/base", 0.0f, 0.25f);
    if (!LoadObjMesh(restPath, restMesh, cancelled, restProgress, &err)) {
        result = {};
        result.message = err.empty() ? "Failed to load rest/base OBJ" : err;
        return false;
    }

    err.clear();
    auto targetProgress = loadProgress("target/detail", 0.25f, 0.25f);
    if (!LoadObjMesh(targetPath, targetMesh, cancelled, targetProgress, &err)) {
        result = {};
        result.message = err.empty() ? "Failed to load target/detail OBJ" : err;
        return false;
    }

    ApplyAxisTransform(restMesh, settings.axisTransform);
    ApplyAxisTransform(targetMesh, settings.axisTransform);

    std::vector<VDMPixel> pixels;
    if (!BakeTwoMeshToPixels(restMesh, targetMesh, settings, pixels, result, cancelled, progress)) return false;
    const std::string bakeSummary = result.message;

    if (Cancelled(cancelled)) {
        result.success = false;
        result.message = "Cancelled";
        return false;
    }

    Report(progress, 0.99f, "Writing EXR");
    if (!SaveEXR32F(outputPath, pixels, settings.width, settings.height, &err)) {
        result.success = false;
        result.message = err.empty() ? "Failed to write EXR output" : err;
        return false;
    }

    result.success = true;
    std::ostringstream ss;
    ss << "Wrote " << PathForMessage(outputPath) << " (" << settings.width << "x" << settings.height
       << ", covered " << result.coveredPixels << " pixels)";
    if (!bakeSummary.empty()) ss << "; " << bakeSummary;
    result.message = ss.str();
    Report(progress, 1.0f, "Done");
    return true;
}

enum ControlId {
    IDC_REST_EDIT = 1001,
    IDC_REST_BROWSE,
    IDC_TARGET_EDIT,
    IDC_TARGET_BROWSE,
    IDC_OUTPUT_EDIT,
    IDC_OUTPUT_BROWSE,
    IDC_MODE,
    IDC_WIDTH,
    IDC_HEIGHT,
    IDC_PLANE,
    IDC_AXIS,
    IDC_VECTOR,
    IDC_PADDING,
    IDC_BASE_AXIS,
    IDC_SCALE,
    IDC_NORMALIZE,
    IDC_FLIPV,
    IDC_RUN,
    IDC_CANCEL,
    IDC_PROGRESS,
    IDC_LOG,
};

struct AppState {
    HWND hwnd = nullptr;
    HWND restEdit = nullptr;
    HWND restBrowse = nullptr;
    HWND targetEdit = nullptr;
    HWND targetBrowse = nullptr;
    HWND outputEdit = nullptr;
    HWND outputBrowse = nullptr;
    HWND modeCombo = nullptr;
    HWND widthEdit = nullptr;
    HWND heightEdit = nullptr;
    HWND planeCombo = nullptr;
    HWND axisCombo = nullptr;
    HWND vectorCombo = nullptr;
    HWND paddingEdit = nullptr;
    HWND baseAxisEdit = nullptr;
    HWND scaleEdit = nullptr;
    HWND normalizeCheck = nullptr;
    HWND flipVCheck = nullptr;
    HWND runButton = nullptr;
    HWND cancelButton = nullptr;
    HWND progress = nullptr;
    HWND log = nullptr;
    HFONT font = nullptr;
    std::atomic<bool> cancel{false};
    bool running = false;
    std::thread worker;
    std::string lastLogStatus;
};

AppState g_app;

void SetControlFont(HWND h) {
    if (h && g_app.font) SendMessageW(h, WM_SETFONT, reinterpret_cast<WPARAM>(g_app.font), TRUE);
}

HWND MakeControl(HWND parent, const wchar_t* cls, const wchar_t* text, DWORD style, int x, int y, int w, int h, int id) {
    HWND ctrl = CreateWindowExW(0, cls, text, style, x, y, w, h, parent, reinterpret_cast<HMENU>(static_cast<intptr_t>(id)), GetModuleHandleW(nullptr), nullptr);
    SetControlFont(ctrl);
    return ctrl;
}

HWND MakeEdit(HWND parent, const wchar_t* text, int x, int y, int w, int h, int id) {
    return MakeControl(parent, L"EDIT", text, WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL, x, y, w, h, id);
}

HWND MakeButton(HWND parent, const wchar_t* text, int x, int y, int w, int h, int id) {
    return MakeControl(parent, L"BUTTON", text, WS_CHILD | WS_VISIBLE | WS_TABSTOP, x, y, w, h, id);
}

HWND MakeLabel(HWND parent, const wchar_t* text, int x, int y, int w, int h) {
    return MakeControl(parent, L"STATIC", text, WS_CHILD | WS_VISIBLE, x, y, w, h, 0);
}

HWND MakeCombo(HWND parent, int x, int y, int w, int h, int id) {
    return MakeControl(parent, L"COMBOBOX", L"", WS_CHILD | WS_VISIBLE | WS_TABSTOP | CBS_DROPDOWNLIST, x, y, w, h, id);
}

std::wstring GetText(HWND h) {
    int n = GetWindowTextLengthW(h);
    std::wstring text(static_cast<size_t>(n) + 1u, L'\0');
    GetWindowTextW(h, text.data(), n + 1);
    text.resize(static_cast<size_t>(std::wcslen(text.c_str())));
    return text;
}

void SetText(HWND h, const std::wstring& text) {
    SetWindowTextW(h, text.c_str());
}

int GetComboIndex(HWND h, int fallback = 0) {
    LRESULT v = SendMessageW(h, CB_GETCURSEL, 0, 0);
    return v == CB_ERR ? fallback : static_cast<int>(v);
}

int GetIntFromEdit(HWND h, int fallback) {
    std::wstring s = GetText(h);
    wchar_t* end = nullptr;
    long v = std::wcstol(s.c_str(), &end, 10);
    return end == s.c_str() ? fallback : static_cast<int>(v);
}

float GetFloatFromEdit(HWND h, float fallback) {
    std::wstring s = GetText(h);
    wchar_t* end = nullptr;
    double v = std::wcstod(s.c_str(), &end);
    return end == s.c_str() ? fallback : static_cast<float>(v);
}

bool IsChecked(HWND h) {
    return SendMessageW(h, BM_GETCHECK, 0, 0) == BST_CHECKED;
}

void AppendLog(const std::string& text) {
    if (!g_app.log) return;
    std::wstring w = Utf8ToWide(text + "\r\n");
    int len = GetWindowTextLengthW(g_app.log);
    SendMessageW(g_app.log, EM_SETSEL, len, len);
    SendMessageW(g_app.log, EM_REPLACESEL, FALSE, reinterpret_cast<LPARAM>(w.c_str()));
}

void SetRunning(bool running) {
    g_app.running = running;
    EnableWindow(g_app.runButton, !running);
    EnableWindow(g_app.cancelButton, running);
    EnableWindow(g_app.restEdit, !running);
    EnableWindow(g_app.restBrowse, !running);
    EnableWindow(g_app.outputEdit, !running);
    EnableWindow(g_app.outputBrowse, !running);
    EnableWindow(g_app.modeCombo, !running);
    EnableWindow(g_app.widthEdit, !running);
    EnableWindow(g_app.heightEdit, !running);
    EnableWindow(g_app.planeCombo, !running);
    EnableWindow(g_app.axisCombo, !running);
    EnableWindow(g_app.vectorCombo, !running);
    EnableWindow(g_app.paddingEdit, !running);
    EnableWindow(g_app.baseAxisEdit, !running);
    EnableWindow(g_app.scaleEdit, !running);
    EnableWindow(g_app.normalizeCheck, !running);
    EnableWindow(g_app.flipVCheck, !running);
    const bool twoMesh = GetComboIndex(g_app.modeCombo) == static_cast<int>(BakeMode::TwoMesh);
    EnableWindow(g_app.targetEdit, !running && twoMesh);
    EnableWindow(g_app.targetBrowse, !running && twoMesh);
}

void UpdateModeState() {
    if (g_app.running) return;
    const bool twoMesh = GetComboIndex(g_app.modeCombo) == static_cast<int>(BakeMode::TwoMesh);
    EnableWindow(g_app.targetEdit, twoMesh);
    EnableWindow(g_app.targetBrowse, twoMesh);
    EnableWindow(g_app.normalizeCheck, twoMesh);
    EnableWindow(g_app.flipVCheck, twoMesh);
    if (!twoMesh) {
        SendMessageW(g_app.normalizeCheck, BM_SETCHECK, BST_UNCHECKED, 0);
        SendMessageW(g_app.flipVCheck, BM_SETCHECK, BST_UNCHECKED, 0);
    }
}

void PostProgress(HWND hwnd, float progress, const std::string& text) {
    auto* msg = new ProgressMsg;
    msg->percent = static_cast<int>(std::clamp(progress, 0.0f, 1.0f) * 100.0f + 0.5f);
    msg->text = text;
    PostMessageW(hwnd, WM_BAKE_PROGRESS, 0, reinterpret_cast<LPARAM>(msg));
}

void PostDone(HWND hwnd, bool ok, const std::string& text) {
    auto* msg = new DoneMsg;
    msg->ok = ok;
    msg->text = text;
    PostMessageW(hwnd, WM_BAKE_DONE, 0, reinterpret_cast<LPARAM>(msg));
}

bool OpenObjDialog(HWND hwnd, std::wstring& pathOut) {
    wchar_t fileName[32768] = {};
    OPENFILENAMEW ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = hwnd;
    ofn.lpstrFilter = L"Wavefront OBJ (*.obj)\0*.obj\0All files (*.*)\0*.*\0";
    ofn.lpstrFile = fileName;
    ofn.nMaxFile = static_cast<DWORD>(std::size(fileName));
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;
    if (!GetOpenFileNameW(&ofn)) return false;
    pathOut = fileName;
    return true;
}

bool SaveExrDialog(HWND hwnd, std::wstring& pathOut) {
    wchar_t fileName[32768] = {};
    OPENFILENAMEW ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = hwnd;
    ofn.lpstrFilter = L"OpenEXR RGBA32F (*.exr)\0*.exr\0All files (*.*)\0*.*\0";
    ofn.lpstrDefExt = L"exr";
    ofn.lpstrFile = fileName;
    ofn.nMaxFile = static_cast<DWORD>(std::size(fileName));
    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST;
    if (!GetSaveFileNameW(&ofn)) return false;
    pathOut = fileName;
    fs::path p(pathOut);
    if (p.extension().empty()) {
        p.replace_extension(L".exr");
        pathOut = p.wstring();
    }
    return true;
}

void SetDefaultOutputFromInput() {
    if (GetText(g_app.outputEdit).empty()) {
        fs::path p(GetText(g_app.restEdit));
        if (!p.empty()) {
            p.replace_filename(p.stem().wstring() + L"_vdm.exr");
            SetText(g_app.outputEdit, p.wstring());
        }
    }
}

BakeSettings ReadSettings() {
    BakeSettings s;
    s.mode = static_cast<BakeMode>(GetComboIndex(g_app.modeCombo, 0));
    s.width = std::clamp(GetIntFromEdit(g_app.widthEdit, 1024), 1, 32768);
    s.height = std::clamp(GetIntFromEdit(g_app.heightEdit, 1024), 1, 32768);
    s.basePlane = static_cast<BasePlane>(GetComboIndex(g_app.planeCombo, 0));
    s.axisTransform = static_cast<AxisTransform>(GetComboIndex(g_app.axisCombo, 2));
    s.vectorMode = static_cast<VectorMode>(GetComboIndex(g_app.vectorCombo, 0));
    s.edgePadding = std::clamp(GetIntFromEdit(g_app.paddingEdit, 4), 0, 512);
    s.baseAxisValue = GetFloatFromEdit(g_app.baseAxisEdit, 0.0f);
    s.outputScale = GetFloatFromEdit(g_app.scaleEdit, 1.0f);
    s.normalizeObjUVs = IsChecked(g_app.normalizeCheck);
    s.flipV = IsChecked(g_app.flipVCheck);

    if (s.mode == BakeMode::SingleExactUv) {
        // Exact UV mode should preserve the sculpt plane UV domain. Normalizing or
        // flipping here silently changes the meaning of the VDM.
        s.normalizeObjUVs = false;
        s.flipV = false;
    }
    return s;
}

void RunBake() {
    if (g_app.running) return;

    const std::wstring restPathW = GetText(g_app.restEdit);
    const std::wstring targetPathW = GetText(g_app.targetEdit);
    std::wstring outputPathW = GetText(g_app.outputEdit);
    BakeSettings settings = ReadSettings();

    if (restPathW.empty() || !fs::exists(fs::path(restPathW))) {
        MessageBoxW(g_app.hwnd, L"Please choose an existing rest/single OBJ file.", L"VDM Baker", MB_OK | MB_ICONWARNING);
        return;
    }
    if (settings.mode == BakeMode::TwoMesh && (targetPathW.empty() || !fs::exists(fs::path(targetPathW)))) {
        MessageBoxW(g_app.hwnd, L"Please choose an existing target/detail OBJ file for two-mesh mode.", L"VDM Baker", MB_OK | MB_ICONWARNING);
        return;
    }
    if (outputPathW.empty()) {
        MessageBoxW(g_app.hwnd, L"Please choose an output EXR path.", L"VDM Baker", MB_OK | MB_ICONWARNING);
        return;
    }
    fs::path outPath(outputPathW);
    if (outPath.extension().empty()) {
        outPath.replace_extension(L".exr");
        outputPathW = outPath.wstring();
        SetText(g_app.outputEdit, outputPathW);
    }

    SetWindowTextW(g_app.log, L"");
    SendMessageW(g_app.progress, PBM_SETPOS, 0, 0);
    g_app.lastLogStatus.clear();
    g_app.cancel = false;
    SetRunning(true);

    AppendLog("Usage: keep original sculpt-plane/rest UVs through the whole workflow. Do not regenerate or re-unwrap UVs after sculpting.");
    if (settings.mode == BakeMode::TwoMesh) {
        AppendLog("Two-mesh mode: rest/base OBJ defines the 2D domain and base positions; target/detail OBJ supplies displaced positions. Target UVs are ignored.");
    } else {
        AppendLog("Single exact UV mode: OBJ faces must reference complete vt UVs, for example f v/vt[/vn].");
    }

    const fs::path restPath(restPathW);
    const fs::path targetPath(targetPathW);
    const fs::path outputPath(outputPathW);
    HWND hwnd = g_app.hwnd;

    g_app.worker = std::thread([settings, restPath, targetPath, outputPath, hwnd]() {
        BakeResult result;
        auto progress = [hwnd](float p, const std::string& text) {
            PostProgress(hwnd, p, text);
        };

        bool ok = false;
        try {
            if (settings.mode == BakeMode::TwoMesh) {
                ok = BakeFromOBJPair(restPath, targetPath, outputPath, settings, result, &g_app.cancel, progress);
            } else {
                ok = BakeFromSingleOBJ(restPath, outputPath, settings, result, &g_app.cancel, progress);
            }
        } catch (const std::exception& e) {
            result.success = false;
            result.message = std::string("Error: ") + e.what();
            ok = false;
        } catch (...) {
            result.success = false;
            result.message = "Unknown error";
            ok = false;
        }
        PostDone(hwnd, ok, result.message);
    });
}

void CancelBake() {
    if (g_app.running) {
        g_app.cancel = true;
        AppendLog("Cancelling...");
    }
}

void CreateUi(HWND hwnd) {
    g_app.hwnd = hwnd;
    g_app.font = reinterpret_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));

    const int labelW = 110;
    const int editX = 130;
    const int editW = 610;
    const int browseX = 750;

    MakeLabel(hwnd, L"Rest/Single OBJ:", 12, 14, labelW, 22);
    g_app.restEdit = MakeEdit(hwnd, L"", editX, 12, editW, 24, IDC_REST_EDIT);
    g_app.restBrowse = MakeButton(hwnd, L"Browse...", browseX, 11, 95, 26, IDC_REST_BROWSE);

    MakeLabel(hwnd, L"Target OBJ:", 12, 45, labelW, 22);
    g_app.targetEdit = MakeEdit(hwnd, L"", editX, 43, editW, 24, IDC_TARGET_EDIT);
    g_app.targetBrowse = MakeButton(hwnd, L"Browse...", browseX, 42, 95, 26, IDC_TARGET_BROWSE);

    MakeLabel(hwnd, L"Output EXR:", 12, 76, labelW, 22);
    g_app.outputEdit = MakeEdit(hwnd, L"", editX, 74, editW, 24, IDC_OUTPUT_EDIT);
    g_app.outputBrowse = MakeButton(hwnd, L"Browse...", browseX, 73, 95, 26, IDC_OUTPUT_BROWSE);

    MakeLabel(hwnd, L"Mode:", 12, 118, 80, 22);
    g_app.modeCombo = MakeCombo(hwnd, 130, 114, 280, 180, IDC_MODE);
    SendMessageW(g_app.modeCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Single mesh: exact UV XYZ"));
    SendMessageW(g_app.modeCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Two meshes: rest/base -> target/detail"));
    SendMessageW(g_app.modeCombo, CB_SETCURSEL, 0, 0);

    MakeLabel(hwnd, L"Width:", 430, 118, 48, 22);
    g_app.widthEdit = MakeEdit(hwnd, L"1024", 480, 114, 80, 24, IDC_WIDTH);
    MakeLabel(hwnd, L"Height:", 575, 118, 55, 22);
    g_app.heightEdit = MakeEdit(hwnd, L"1024", 635, 114, 80, 24, IDC_HEIGHT);

    MakeLabel(hwnd, L"Domain plane:", 12, 152, 100, 22);
    g_app.planeCombo = MakeCombo(hwnd, 130, 148, 210, 160, IDC_PLANE);
    SendMessageW(g_app.planeCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"XY (Z displacement)"));
    SendMessageW(g_app.planeCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"XZ (Y displacement)"));
    SendMessageW(g_app.planeCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"ZY (X displacement)"));
    SendMessageW(g_app.planeCombo, CB_SETCURSEL, 0, 0);

    MakeLabel(hwnd, L"Axis transform:", 365, 152, 105, 22);
    g_app.axisCombo = MakeCombo(hwnd, 480, 148, 260, 160, IDC_AXIS);
    SendMessageW(g_app.axisCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"None"));
    SendMessageW(g_app.axisCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Swap Y/Z"));
    SendMessageW(g_app.axisCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Blender OBJ default (X, -Z, Y)"));
    SendMessageW(g_app.axisCombo, CB_SETCURSEL, 2, 0);

    MakeLabel(hwnd, L"Vector mode:", 12, 186, 100, 22);
    g_app.vectorCombo = MakeCombo(hwnd, 130, 182, 210, 160, IDC_VECTOR);
    SendMessageW(g_app.vectorCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Displacement from base/rest"));
    SendMessageW(g_app.vectorCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Object-space position"));
    SendMessageW(g_app.vectorCombo, CB_SETCURSEL, 0, 0);

    MakeLabel(hwnd, L"Edge padding:", 365, 186, 105, 22);
    g_app.paddingEdit = MakeEdit(hwnd, L"4", 480, 182, 80, 24, IDC_PADDING);
    MakeLabel(hwnd, L"Base axis:", 575, 186, 80, 22);
    g_app.baseAxisEdit = MakeEdit(hwnd, L"0.0", 650, 182, 90, 24, IDC_BASE_AXIS);

    MakeLabel(hwnd, L"Output scale:", 12, 220, 100, 22);
    g_app.scaleEdit = MakeEdit(hwnd, L"1.0", 130, 216, 80, 24, IDC_SCALE);
    g_app.normalizeCheck = MakeControl(hwnd, L"BUTTON", L"Normalize OBJ UVs (two-mesh only)",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX, 240, 218, 230, 22, IDC_NORMALIZE);
    g_app.flipVCheck = MakeControl(hwnd, L"BUTTON", L"Flip V (two-mesh only)",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX, 485, 218, 180, 22, IDC_FLIPV);

    MakeLabel(hwnd,
        L"Pipeline note: keep original rest/sculpt-plane UVs. Do not regenerate, unwrap, or destroy UVs after sculpting.",
        12, 255, 835, 22);

    g_app.progress = MakeControl(hwnd, PROGRESS_CLASSW, L"", WS_CHILD | WS_VISIBLE, 12, 285, 833, 20, IDC_PROGRESS);
    SendMessageW(g_app.progress, PBM_SETRANGE, 0, MAKELPARAM(0, 100));

    g_app.log = MakeControl(hwnd, L"EDIT", L"",
        WS_CHILD | WS_VISIBLE | WS_BORDER | WS_VSCROLL | ES_MULTILINE | ES_READONLY | ES_AUTOVSCROLL,
        12, 315, 833, 245, IDC_LOG);

    g_app.runButton = MakeButton(hwnd, L"Run", 12, 575, 90, 30, IDC_RUN);
    g_app.cancelButton = MakeButton(hwnd, L"Cancel", 110, 575, 90, 30, IDC_CANCEL);
    EnableWindow(g_app.cancelButton, FALSE);

    AppendLog("Ready. This standalone app writes uncompressed RGBA32F EXR files.");
    AppendLog("Single exact UV mode needs complete OBJ UVs. Two-mesh mode uses rest/base UVs; target/detail UVs are ignored.");
    UpdateModeState();
}

void ResizeUi(HWND hwnd) {
    RECT rc{};
    GetClientRect(hwnd, &rc);
    const int w = rc.right - rc.left;
    const int h = rc.bottom - rc.top;
    if (w <= 300 || h <= 350) return;

    const int rightPad = 12;
    const int browseW = 95;
    const int browseX = w - rightPad - browseW;
    const int editX = 130;
    const int editW = browseX - editX - 10;
    MoveWindow(g_app.restEdit, editX, 12, editW, 24, TRUE);
    MoveWindow(g_app.restBrowse, browseX, 11, browseW, 26, TRUE);
    MoveWindow(g_app.targetEdit, editX, 43, editW, 24, TRUE);
    MoveWindow(g_app.targetBrowse, browseX, 42, browseW, 26, TRUE);
    MoveWindow(g_app.outputEdit, editX, 74, editW, 24, TRUE);
    MoveWindow(g_app.outputBrowse, browseX, 73, browseW, 26, TRUE);

    MoveWindow(g_app.progress, 12, 285, w - 24, 20, TRUE);
    MoveWindow(g_app.log, 12, 315, w - 24, std::max(120, h - 385), TRUE);
    MoveWindow(g_app.runButton, 12, h - 43, 90, 30, TRUE);
    MoveWindow(g_app.cancelButton, 110, h - 43, 90, 30, TRUE);
}

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
        case WM_CREATE:
            CreateUi(hwnd);
            return 0;

        case WM_SIZE:
            ResizeUi(hwnd);
            return 0;

        case WM_COMMAND: {
            const int id = LOWORD(wParam);
            const int code = HIWORD(wParam);
            if (id == IDC_REST_BROWSE) {
                std::wstring p;
                if (OpenObjDialog(hwnd, p)) {
                    SetText(g_app.restEdit, p);
                    SetDefaultOutputFromInput();
                }
                return 0;
            }
            if (id == IDC_TARGET_BROWSE) {
                std::wstring p;
                if (OpenObjDialog(hwnd, p)) SetText(g_app.targetEdit, p);
                return 0;
            }
            if (id == IDC_OUTPUT_BROWSE) {
                std::wstring p;
                if (SaveExrDialog(hwnd, p)) SetText(g_app.outputEdit, p);
                return 0;
            }
            if (id == IDC_RUN) {
                RunBake();
                return 0;
            }
            if (id == IDC_CANCEL) {
                CancelBake();
                return 0;
            }
            if (id == IDC_MODE && code == CBN_SELCHANGE) {
                UpdateModeState();
                return 0;
            }
            return 0;
        }

        case WM_BAKE_PROGRESS: {
            std::unique_ptr<ProgressMsg> p(reinterpret_cast<ProgressMsg*>(lParam));
            SendMessageW(g_app.progress, PBM_SETPOS, p->percent, 0);
            if (!p->text.empty() && p->text != g_app.lastLogStatus) {
                g_app.lastLogStatus = p->text;
                AppendLog(p->text);
            }
            return 0;
        }

        case WM_BAKE_DONE: {
            std::unique_ptr<DoneMsg> done(reinterpret_cast<DoneMsg*>(lParam));
            if (g_app.worker.joinable()) g_app.worker.join();
            if (done->ok) SendMessageW(g_app.progress, PBM_SETPOS, 100, 0);
            AppendLog(std::string(done->ok ? "[OK] " : "[FAIL] ") + done->text);
            SetRunning(false);
            return 0;
        }

        case WM_CLOSE:
            if (g_app.running) {
                if (MessageBoxW(hwnd, L"A bake is running. Cancel and close?", L"VDM Baker", MB_YESNO | MB_ICONQUESTION) != IDYES) {
                    return 0;
                }
                g_app.cancel = true;
                if (g_app.worker.joinable()) g_app.worker.join();
            }
            DestroyWindow(hwnd);
            return 0;

        case WM_DESTROY:
            if (g_app.worker.joinable()) g_app.worker.join();
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

} // namespace

int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE, PWSTR, int nCmdShow) {
    INITCOMMONCONTROLSEX icc{};
    icc.dwSize = sizeof(icc);
    icc.dwICC = ICC_PROGRESS_CLASS;
    InitCommonControlsEx(&icc);

    const wchar_t* className = L"StandaloneVDMBaker";
    WNDCLASSEXW wc{};
    wc.cbSize = sizeof(wc);
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    wc.hIcon = LoadIconW(nullptr, IDI_APPLICATION);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_BTNFACE + 1);
    wc.lpszClassName = className;
    RegisterClassExW(&wc);

    HWND hwnd = CreateWindowExW(
        0,
        className,
        L"Standalone VDM Baker (OBJ to RGBA32F EXR)",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        880,
        660,
        nullptr,
        nullptr,
        hInstance,
        nullptr);

    if (!hwnd) return 1;

    ShowWindow(hwnd, nCmdShow);
    UpdateWindow(hwnd);

    MSG msg{};
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return static_cast<int>(msg.wParam);
}
