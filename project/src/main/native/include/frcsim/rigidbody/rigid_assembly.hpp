#pragma once
#include "frcsim/rigidbody/rigid_body.hpp"
#include <vector>
namespace frcsim {
class RigidAssembly {
  public:
    std::vector<RigidBody*> bodies;
};
}
