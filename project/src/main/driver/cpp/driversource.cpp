// RenSim physics driver — self-contained C API implementation.
// Implements driverheader.h without depending on the RenSim C++ physics-core
// headers (which are not fully committed to the repository yet).
// Uses semi-implicit Euler integration identical to frcsim::PhysicsWorld::step().

#include "driverheader.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace {

struct Body {
    double x_m{0.0};
    double y_m{0.0};
    double z_m{0.0};
    double vx_mps{0.0};
    double vy_mps{0.0};
    double vz_mps{0.0};
    double mass_kg{1.0};
    bool gravity_enabled{true};
};

struct World {
    double fixed_dt_s{0.005};
    bool enable_gravity{false};
    double gravity_x_mps2{0.0};
    double gravity_y_mps2{0.0};
    double gravity_z_mps2{-9.80665};
    std::vector<Body> bodies;
};

std::mutex g_mutex;
std::unordered_map<std::uint64_t, std::unique_ptr<World>> g_worlds;
std::uint64_t g_next_handle{1};

World* getWorld(std::uint64_t handle) {
    const auto it = g_worlds.find(handle);
    return (it != g_worlds.end()) ? it->second.get() : nullptr;
}

Body* getBody(World* world, int index) {
    if (!world || index < 0) return nullptr;
    const std::size_t idx = static_cast<std::size_t>(index);
    if (idx >= world->bodies.size()) return nullptr;
    return &world->bodies[idx];
}

}  // namespace

extern "C" {

void c_doThing() {}

unsigned long long c_rsCreateWorld(double fixed_dt_s, int enable_gravity) {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto world = std::make_unique<World>();
    world->fixed_dt_s = (fixed_dt_s > 0.0) ? fixed_dt_s : 0.005;
    world->enable_gravity = (enable_gravity != 0);
    const std::uint64_t handle = g_next_handle++;
    g_worlds.emplace(handle, std::move(world));
    return handle;
}

void c_rsDestroyWorld(unsigned long long world_handle) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_worlds.erase(static_cast<std::uint64_t>(world_handle));
}

int c_rsCreateBody(unsigned long long world_handle, double mass_kg) {
    std::lock_guard<std::mutex> lock(g_mutex);
    World* world = getWorld(static_cast<std::uint64_t>(world_handle));
    if (!world) return -1;
    Body body;
    body.mass_kg = (mass_kg > 0.0) ? mass_kg : 1.0;
    world->bodies.push_back(body);
    return static_cast<int>(world->bodies.size() - 1);
}

int c_rsSetBodyPosition(unsigned long long world_handle, int body_index,
                        double x_m, double y_m, double z_m) {
    std::lock_guard<std::mutex> lock(g_mutex);
    Body* body = getBody(getWorld(static_cast<std::uint64_t>(world_handle)), body_index);
    if (!body) return -1;
    body->x_m = x_m;
    body->y_m = y_m;
    body->z_m = z_m;
    return 0;
}

int c_rsSetBodyLinearVelocity(unsigned long long world_handle, int body_index,
                              double vx_mps, double vy_mps, double vz_mps) {
    std::lock_guard<std::mutex> lock(g_mutex);
    Body* body = getBody(getWorld(static_cast<std::uint64_t>(world_handle)), body_index);
    if (!body) return -1;
    body->vx_mps = vx_mps;
    body->vy_mps = vy_mps;
    body->vz_mps = vz_mps;
    return 0;
}

int c_rsSetBodyGravityEnabled(unsigned long long world_handle, int body_index,
                              int enabled) {
    std::lock_guard<std::mutex> lock(g_mutex);
    Body* body = getBody(getWorld(static_cast<std::uint64_t>(world_handle)), body_index);
    if (!body) return -1;
    body->gravity_enabled = (enabled != 0);
    return 0;
}

int c_rsSetWorldGravity(unsigned long long world_handle,
                        double gx_mps2, double gy_mps2, double gz_mps2) {
    std::lock_guard<std::mutex> lock(g_mutex);
    World* world = getWorld(static_cast<std::uint64_t>(world_handle));
    if (!world) return -1;
    world->gravity_x_mps2 = gx_mps2;
    world->gravity_y_mps2 = gy_mps2;
    world->gravity_z_mps2 = gz_mps2;
    world->enable_gravity = true;
    return 0;
}

int c_rsStepWorld(unsigned long long world_handle, int steps) {
    std::lock_guard<std::mutex> lock(g_mutex);
    World* world = getWorld(static_cast<std::uint64_t>(world_handle));
    if (!world) return -1;
    const int safe_steps = std::max(steps, 1);
    const double dt = world->fixed_dt_s;
    for (int s = 0; s < safe_steps; ++s) {
        for (Body& body : world->bodies) {
            // Semi-implicit Euler: update velocity first, then position.
            // Mirrors frcsim::Integrator::integrateLinear().
            if (world->enable_gravity && body.gravity_enabled) {
                body.vx_mps += world->gravity_x_mps2 * dt;
                body.vy_mps += world->gravity_y_mps2 * dt;
                body.vz_mps += world->gravity_z_mps2 * dt;
            }
            body.x_m += body.vx_mps * dt;
            body.y_m += body.vy_mps * dt;
            body.z_m += body.vz_mps * dt;
        }
    }
    return 0;
}

int c_rsGetBodyPosition(unsigned long long world_handle, int body_index,
                        double* x_m, double* y_m, double* z_m) {
    if (!x_m || !y_m || !z_m) return -1;
    std::lock_guard<std::mutex> lock(g_mutex);
    const Body* body = getBody(getWorld(static_cast<std::uint64_t>(world_handle)), body_index);
    if (!body) return -1;
    *x_m = body->x_m;
    *y_m = body->y_m;
    *z_m = body->z_m;
    return 0;
}

int c_rsGetBodyLinearVelocity(unsigned long long world_handle, int body_index,
                              double* vx_mps, double* vy_mps, double* vz_mps) {
    if (!vx_mps || !vy_mps || !vz_mps) return -1;
    std::lock_guard<std::mutex> lock(g_mutex);
    const Body* body = getBody(getWorld(static_cast<std::uint64_t>(world_handle)), body_index);
    if (!body) return -1;
    *vx_mps = body->vx_mps;
    *vy_mps = body->vy_mps;
    *vz_mps = body->vz_mps;
    return 0;
}

}  // extern "C"
