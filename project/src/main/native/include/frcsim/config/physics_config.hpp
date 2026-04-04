#pragma once
namespace frcsim {
struct PhysicsConfig {
    double fixed_dt_s{0.005};
    bool enable_gravity{true};
};
}
