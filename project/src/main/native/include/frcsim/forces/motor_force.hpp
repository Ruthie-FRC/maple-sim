#pragma once
#include "frcsim/forces/force_generator.hpp"
namespace frcsim {
class MotorForce : public ForceGenerator {
  public:
    void apply(RigidBody& body, double dt_s) override;
};
}
