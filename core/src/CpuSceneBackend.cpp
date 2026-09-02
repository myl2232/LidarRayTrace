#include "lidar_core/CpuSceneBackend.h"

#include <algorithm>
#include <limits>

namespace lidar {
namespace {

bool intersectPlane(const RayRequest& ray, const PlanePrimitive& plane, HitResult& hit) {
    const float denom = dot(plane.normal, ray.direction);
    if (std::fabs(denom) < 1e-8f) {
        return false;
    }
    const float t = dot(plane.point - ray.origin, plane.normal) / denom;
    if (t < ray.t_min || t > ray.t_max) {
        return false;
    }
    hit.hit = true;
    hit.t = t;
    hit.point = ray.origin + ray.direction * t;
    hit.normal = denom < 0.0f ? plane.normal : (plane.normal * -1.0f);
    hit.intensity = plane.reflectivity * 255.0f;
    return true;
}

bool intersectSphere(const RayRequest& ray, const SpherePrimitive& sphere, HitResult& hit) {
    const Vec3 oc = ray.origin - sphere.center;
    const float b = dot(oc, ray.direction);
    const float c = dot(oc, oc) - sphere.radius * sphere.radius;
    const float disc = b * b - c;
    if (disc < 0.0f) {
        return false;
    }
    const float s = std::sqrt(disc);
    float t = -b - s;
    if (t < ray.t_min) {
        t = -b + s;
    }
    if (t < ray.t_min || t > ray.t_max) {
        return false;
    }
    hit.hit = true;
    hit.t = t;
    hit.point = ray.origin + ray.direction * t;
    hit.normal = (hit.point - sphere.center).normalized();
    hit.intensity = sphere.reflectivity * 255.0f;
    return true;
}

bool intersectAabb(const RayRequest& ray, const AabbPrimitive& box, HitResult& hit) {
    float tmin = ray.t_min;
    float tmax = ray.t_max;
    Vec3 n{0.0f, 0.0f, 1.0f};

    const float orig[3] = {ray.origin.x, ray.origin.y, ray.origin.z};
    const float dir[3] = {ray.direction.x, ray.direction.y, ray.direction.z};
    const float bmin[3] = {box.min.x, box.min.y, box.min.z};
    const float bmax[3] = {box.max.x, box.max.y, box.max.z};

    for (int i = 0; i < 3; ++i) {
        if (std::fabs(dir[i]) < 1e-12f) {
            if (orig[i] < bmin[i] || orig[i] > bmax[i]) {
                return false;
            }
            continue;
        }
        const float inv = 1.0f / dir[i];
        float t0 = (bmin[i] - orig[i]) * inv;
        float t1 = (bmax[i] - orig[i]) * inv;
        float sign = -1.0f;
        if (t0 > t1) {
            std::swap(t0, t1);
            sign = 1.0f;
        }
        if (t0 > tmin) {
            tmin = t0;
            n = Vec3{0.0f, 0.0f, 0.0f};
            if (i == 0) {
                n.x = sign;
            } else if (i == 1) {
                n.y = sign;
            } else {
                n.z = sign;
            }
        }
        tmax = std::min(tmax, t1);
        if (tmax < tmin) {
            return false;
        }
    }

    hit.hit = true;
    hit.t = tmin;
    hit.point = ray.origin + ray.direction * tmin;
    hit.normal = n;
    hit.intensity = box.reflectivity * 255.0f;
    return true;
}

void consider(HitResult& best, const HitResult& cand, uint32_t prim) {
    if (cand.hit && (!best.hit || cand.t < best.t)) {
        best = cand;
        best.primitive_id = prim;
    }
}

}  // namespace

void CpuSceneBackend::addBoxRoom(float half_xy, float height, float reflectivity) {
    addPlane(PlanePrimitive{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, reflectivity});
    addPlane(PlanePrimitive{{0.0f, 0.0f, height}, {0.0f, 0.0f, -1.0f}, reflectivity});
    addPlane(PlanePrimitive{{half_xy, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, reflectivity});
    addPlane(PlanePrimitive{{-half_xy, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, reflectivity});
    addPlane(PlanePrimitive{{0.0f, half_xy, 0.0f}, {0.0f, -1.0f, 0.0f}, reflectivity});
    addPlane(PlanePrimitive{{0.0f, -half_xy, 0.0f}, {0.0f, 1.0f, 0.0f}, reflectivity});
}

void CpuSceneBackend::traceBatch(const std::vector<RayRequest>& rays, std::vector<HitResult>& hits) {
    hits.assign(rays.size(), {});
    for (std::size_t i = 0; i < rays.size(); ++i) {
        HitResult best;
        best.t = std::numeric_limits<float>::infinity();
        uint32_t prim = 0;
        for (const PlanePrimitive& p : planes_) {
            HitResult cand;
            if (intersectPlane(rays[i], p, cand)) {
                consider(best, cand, prim);
            }
            ++prim;
        }
        for (const SpherePrimitive& s : spheres_) {
            HitResult cand;
            if (intersectSphere(rays[i], s, cand)) {
                consider(best, cand, prim);
            }
            ++prim;
        }
        for (const AabbPrimitive& b : aabbs_) {
            HitResult cand;
            if (intersectAabb(rays[i], b, cand)) {
                consider(best, cand, prim);
            }
            ++prim;
        }
        hits[i] = best;
    }
}

}  // namespace lidar
