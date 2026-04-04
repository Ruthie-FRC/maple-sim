#pragma once
#include "frcsim/forces/force_generator.hpp"
namespace frcsim {
class Gravity : public ForceGenerator {
  public:
    explicit Gravity(double gz_mps2 = -9.80665);
    void apply(RigidBody& body, double dt_s) override;
  private:
    double gz_mps2_;
};
}
