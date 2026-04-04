#pragma once
namespace frcsim {
class RigidBody {
  public:
    double mass_kg{1.0};
    double x_m{0.0};
    double y_m{0.0};
    double z_m{0.0};
    double vx_mps{0.0};
    double vy_mps{0.0};
    double vz_mps{0.0};
    bool gravity_enabled{true};
};
}
