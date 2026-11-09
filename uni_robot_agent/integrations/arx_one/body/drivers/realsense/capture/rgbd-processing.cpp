// Ordinary-user, memory-only processing. This translation unit creates no
// context/device/pipeline, opens no USB device, and performs no network I/O.
#include <librealsense2/rsutil.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

static bool intrinsics_ok(const rs2_intrinsics* p) {
    return p && p->width > 0 && p->width <= 1920 && p->height > 0 && p->height <= 1080
        && std::isfinite(p->fx) && std::isfinite(p->fy) && p->fx > 0 && p->fy > 0
        && std::isfinite(p->ppx) && std::isfinite(p->ppy)
        && p->model >= RS2_DISTORTION_NONE && p->model < RS2_DISTORTION_COUNT
        && std::all_of(p->coeffs, p->coeffs + 5, [](float x){return std::isfinite(x);});
}

extern "C" int rua_deproject(const rs2_intrinsics* intrinsics,
                                  float u, float v, float z, float* xyz) {
    if (!intrinsics_ok(intrinsics) || !xyz || !std::isfinite(z) || z <= 0
        || !std::isfinite(u) || !std::isfinite(v)
        || intrinsics->model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY) return -1;
    const float pixel[2] = {u, v};
    rs2_deproject_pixel_to_point(xyz, intrinsics, pixel, z);
    return std::all_of(xyz, xyz + 3, [](float x){return std::isfinite(x);}) ? 0 : -2;
}

// Explicit nearest-pixel forward registration, not rs2::align's footprint
// rasterizer. Use SDK distortion/projection/extrinsics, target-optical Z,
// nearest-surface z-buffer; holes remain invalid. No synthetic hole filling.
// Buffer sizes are in bytes; both must cover their intrinsics' width x height Z16 image.
extern "C" int rua_align_z16_sized(const uint16_t* depth, size_t depth_bytes,
                                   uint16_t* aligned, size_t aligned_bytes,
                                   float scale, const rs2_intrinsics* source,
                                   const rs2_intrinsics* target,
                                   const rs2_extrinsics* extrinsics) {
    if (!depth || !aligned || !extrinsics || !intrinsics_ok(source)
        || !intrinsics_ok(target) || !std::isfinite(scale) || scale <= 0
        || source->model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY) return -1;
    if (depth_bytes < size_t(source->width) * source->height * sizeof(uint16_t)
        || aligned_bytes < size_t(target->width) * target->height * sizeof(uint16_t)) return -1;
    // Pinned librealsense 2.58.3 src/rs.cpp explicitly supports projection
    // into INVERSE_BROWN_CONRADY. D405 RGB factory intrinsics use this model.
    // Only source MODIFIED_BROWN remains rejected: SDK deprojection asserts
    // for that model. All distortion math below stays inside the SDK.
    for (float x : extrinsics->rotation) if (!std::isfinite(x)) return -1;
    for (float x : extrinsics->translation) if (!std::isfinite(x)) return -1;
    const size_t count = size_t(target->width) * target->height;
    try {
        std::vector<float> closest(count, INFINITY);
        std::memset(aligned, 0, count * sizeof(uint16_t));
        for (int v = 0; v < source->height; ++v) {
            for (int u = 0; u < source->width; ++u) {
                const uint16_t raw = depth[size_t(v) * source->width + u];
                if (!raw) continue;
                float source_xyz[3], target_xyz[3], pixel[2];
                if (rua_deproject(source, u, v, raw * scale, source_xyz)) continue;
                rs2_transform_point_to_point(target_xyz, extrinsics, source_xyz);
                if (!std::isfinite(target_xyz[2]) || target_xyz[2] <= 0) continue;
                rs2_project_point_to_pixel(pixel, target, target_xyz);
                if (!std::isfinite(pixel[0]) || !std::isfinite(pixel[1])
                    || pixel[0] < -0.5f || pixel[0] >= target->width - 0.5f
                    || pixel[1] < -0.5f || pixel[1] >= target->height - 0.5f) continue;
                const int x = int(std::floor(pixel[0] + 0.5f));
                const int y = int(std::floor(pixel[1] + 0.5f));
                const size_t index = size_t(y) * target->width + x;
                const double value = std::round(double(target_xyz[2]) / scale);
                if (value < 1 || value > 65535 || target_xyz[2] >= closest[index]) continue;
                closest[index] = target_xyz[2];
                aligned[index] = uint16_t(value);
            }
        }
        return 0;
    } catch (...) { return -2; }
}

extern "C" int rua_rgb_to_bgra(const uint8_t* rgb, uint8_t* bgra, size_t count) {
    if (!rgb || !bgra || count > 1920 * 1080) return -1;
    for (size_t i = 0; i < count; ++i) {
        bgra[4*i] = rgb[3*i+2]; bgra[4*i+1] = rgb[3*i+1];
        bgra[4*i+2] = rgb[3*i]; bgra[4*i+3] = 255;
    }
    return 0;
}

extern "C" int rua_depth_preview(const uint16_t* depth, uint8_t* rgb,
                                      size_t count, float scale) {
    if (!depth || !rgb || count > 1920 * 1080 || !std::isfinite(scale) || scale <= 0) return -1;
    for (size_t i = 0; i < count; ++i) {
        const float z = depth[i] * scale;
        uint8_t* out = rgb + 3*i;
        if (!depth[i]) { out[0] = out[1] = out[2] = 0; }
        else if (z < .07f) { out[0] = out[1] = out[2] = 255; }
        else if (z > .50f) { out[0] = out[1] = out[2] = 80; }
        else {
            const float t = (z - .07f) / .43f;
            out[0] = uint8_t(255 * std::max(0.f, 2*t - 1));
            out[1] = uint8_t(255 * (1 - std::abs(2*t - 1)));
            out[2] = uint8_t(255 * std::max(0.f, 1 - 2*t));
        }
    }
    return 0;
}
