#include "header.h"
#include "driverheader.h"
namespace rensim {
PhysicsWorld::PhysicsWorld(double fixed_dt_seconds, bool enable_gravity)
    : handle_(c_rsCreateWorld(fixed_dt_seconds, enable_gravity ? 1 : 0)) {}
PhysicsWorld::~PhysicsWorld() {
  if (handle_ != 0) { c_rsDestroyWorld(handle_); handle_ = 0; }
}
PhysicsWorld::PhysicsWorld(PhysicsWorld&& other) noexcept { handle_ = other.handle_; other.handle_ = 0; }
PhysicsWorld& PhysicsWorld::operator=(PhysicsWorld&& other) noexcept {
  if (this == &other) return *this;
  if (handle_ != 0) c_rsDestroyWorld(handle_);
  handle_ = other.handle_; other.handle_ = 0; return *this;
}
int PhysicsWorld::createBody(double mass_kg) {
  if (handle_ == 0) return -1;
  return c_rsCreateBody(handle_, mass_kg);
}
bool PhysicsWorld::setBodyPosition(int body_index, const Vec3& p) {
  return handle_ != 0 && c_rsSetBodyPosition(handle_, body_index, p.x, p.y, p.z) == 0;
}
bool PhysicsWorld::setBodyLinearVelocity(int body_index, const Vec3& v) {
  return handle_ != 0 && c_rsSetBodyLinearVelocity(handle_, body_index, v.x, v.y, v.z) == 0;
}
bool PhysicsWorld::setBodyGravityEnabled(int body_index, bool enabled) {
  return handle_ != 0 && c_rsSetBodyGravityEnabled(handle_, body_index, enabled ? 1 : 0) == 0;
}
bool PhysicsWorld::setGravity(const Vec3& g) {
  return handle_ != 0 && c_rsSetWorldGravity(handle_, g.x, g.y, g.z) == 0;
}
bool PhysicsWorld::step(int steps) {
  return handle_ != 0 && c_rsStepWorld(handle_, steps) == 0;
}
bool PhysicsWorld::bodyPosition(int body_index, Vec3* out) const {
  if (handle_ == 0 || out == nullptr) return false;
  return c_rsGetBodyPosition(handle_, body_index, &out->x, &out->y, &out->z) == 0;
}
bool PhysicsWorld::bodyLinearVelocity(int body_index, Vec3* out) const {
  if (handle_ == 0 || out == nullptr) return false;
  return c_rsGetBodyLinearVelocity(handle_, body_index, &out->x, &out->y, &out->z) == 0;
}
}
