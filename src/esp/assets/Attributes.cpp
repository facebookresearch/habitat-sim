// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Attributes.h"

namespace esp {
namespace assets {

//----------------------------------------//
//  Derived attribute implementations
//----------------------------------------//

PhysicsObjectAttributes::PhysicsObjectAttributes() {
  // fill necessary attribute defaults
  setMass(1.0);
  setMargin(0.01);
  setScale({1.0, 1.0, 1.0});
  setCOM({0, 0, 0});
  setInertia({0, 0, 0});
  setFrictionCoefficient(0.5);
  setRestitutionCoefficient(0.1);
  setLinearDamping(0.2);
  setAngularDamping(0.2);
  setOriginHandle("");
  setRenderMeshHandle("");
  setCollisionMeshHandle("");
  setBoundingBoxCollisions(false);
  setJoinCollisionMeshes(true);
  setRequiresLighting(true);
}

// center of mass (COM)
void PhysicsObjectAttributes::setCOM(const Magnum::Vector3& com) {
  setVec3("COM", com);
}
Magnum::Vector3 PhysicsObjectAttributes::getCOM() const {
  return getVec3("COM");
}

// collision shape inflation margin
void PhysicsObjectAttributes::setMargin(double margin) {
  setDouble("margin", margin);
}
double PhysicsObjectAttributes::getMargin() const {
  return getDouble("margin");
}

void PhysicsObjectAttributes::setMass(double mass) {
  setDouble("mass", mass);
}
double PhysicsObjectAttributes::getMass() const {
  return getDouble("mass");
}

// inertia diagonal
void PhysicsObjectAttributes::setInertia(const Magnum::Vector3& inertia) {
  setVec3("inertia", inertia);
}
Magnum::Vector3 PhysicsObjectAttributes::getInertia() const {
  return getVec3("inertia");
}

void PhysicsObjectAttributes::setScale(const Magnum::Vector3& scale) {
  setVec3("scale", scale);
}
Magnum::Vector3 PhysicsObjectAttributes::getScale() const {
  return getVec3("scale");
}

void PhysicsObjectAttributes::setFrictionCoefficient(
    double frictionCoefficient) {
  setDouble("frictionCoefficient", frictionCoefficient);
}
double PhysicsObjectAttributes::getFrictionCoefficient() const {
  return getDouble("frictionCoefficient");
}

void PhysicsObjectAttributes::setRestitutionCoefficient(
    double restitutionCoefficient) {
  setDouble("restitutionCoefficient", restitutionCoefficient);
}
double PhysicsObjectAttributes::getRestitutionCoefficient() const {
  return getDouble("restitutionCoefficient");
}

void PhysicsObjectAttributes::setLinearDamping(double linearDamping) {
  setDouble("linearDamping", linearDamping);
}
double PhysicsObjectAttributes::getLinearDamping() const {
  return getDouble("linearDamping");
}

void PhysicsObjectAttributes::setAngularDamping(double angularDamping) {
  setDouble("angularDamping", angularDamping);
}
double PhysicsObjectAttributes::getAngularDamping() const {
  return getDouble("angularDamping");
}

void PhysicsObjectAttributes::setOriginHandle(const std::string& originHandle) {
  setString("originHandle", originHandle);
}
std::string PhysicsObjectAttributes::getOriginHandle() const {
  return getString("originHandle");
}

void PhysicsObjectAttributes::setRenderMeshHandle(
    const std::string& renderMeshHandle) {
  setString("renderMeshHandle", renderMeshHandle);
}
std::string PhysicsObjectAttributes::getRenderMeshHandle() const {
  return getString("renderMeshHandle");
}

void PhysicsObjectAttributes::setCollisionMeshHandle(
    const std::string& collisionMeshHandle) {
  setString("collisionMeshHandle", collisionMeshHandle);
}
std::string PhysicsObjectAttributes::getCollisionMeshHandle() const {
  return getString("collisionMeshHandle");
}

void PhysicsObjectAttributes::setObjectTemplateID(int objectTemplateID) {
  setInt("objectTemplateID", objectTemplateID);
}

int PhysicsObjectAttributes::getObjectTemplateID() const {
  return getInt("objectTemplateID");
}

// if true override other settings and use render mesh bounding box as collision
// object
void PhysicsObjectAttributes::setBoundingBoxCollisions(
    bool useBoundingBoxForCollision) {
  setBool("useBoundingBoxForCollision", useBoundingBoxForCollision);
}
bool PhysicsObjectAttributes::getBoundingBoxCollisions() const {
  return getBool("useBoundingBoxForCollision");
}

// if true join all mesh components of an asset into a unified collision object
void PhysicsObjectAttributes::setJoinCollisionMeshes(bool joinCollisionMeshes) {
  setBool("joinCollisionMeshes", joinCollisionMeshes);
}
bool PhysicsObjectAttributes::getJoinCollisionMeshes() const {
  return getBool("joinCollisionMeshes");
}

// if true use phong illumination model instead of flat shading
void PhysicsObjectAttributes::setRequiresLighting(bool requiresLighting) {
  setBool("requiresLighting", requiresLighting);
}
bool PhysicsObjectAttributes::getRequiresLighting() const {
  return getBool("requiresLighting");
}

PhysicsSceneAttributes::PhysicsSceneAttributes() {
  setGravity({0, -9.8, 0});
  setFrictionCoefficient(0.4);
  setRestitutionCoefficient(0.05);
  setRenderMeshHandle("");
  setCollisionMeshHandle("");
}

void PhysicsSceneAttributes::setGravity(const Magnum::Vector3& gravity) {
  setVec3("gravity", gravity);
}
Magnum::Vector3 PhysicsSceneAttributes::getGravity() const {
  return getVec3("gravity");
}

void PhysicsSceneAttributes::setFrictionCoefficient(
    double frictionCoefficient) {
  setDouble("frictionCoefficient", frictionCoefficient);
}
double PhysicsSceneAttributes::getFrictionCoefficient() const {
  return getDouble("frictionCoefficient");
}

void PhysicsSceneAttributes::setRestitutionCoefficient(
    double restitutionCoefficient) {
  setDouble("restitutionCoefficient", restitutionCoefficient);
}
double PhysicsSceneAttributes::getRestitutionCoefficient() const {
  return getDouble("restitutionCoefficient");
}

void PhysicsSceneAttributes::setRenderMeshHandle(
    const std::string& renderMeshHandle) {
  setString("renderMeshHandle", renderMeshHandle);
}
std::string PhysicsSceneAttributes::getRenderMeshHandle() const {
  return getString("renderMeshHandle");
}

void PhysicsSceneAttributes::setCollisionMeshHandle(
    const std::string& collisionMeshHandle) {
  setString("collisionMeshHandle", collisionMeshHandle);
}
std::string PhysicsSceneAttributes::getCollisionMeshHandle() const {
  return getString("collisionMeshHandle");
}

PhysicsManagerAttributes::PhysicsManagerAttributes() {
  setSimulator("none");
  setTimestep(0.01);
  setMaxSubsteps(10);
}

void PhysicsManagerAttributes::setSimulator(const std::string& simulator) {
  setString("simulator", simulator);
}
std::string PhysicsManagerAttributes::getSimulator() const {
  return getString("simulator");
}

void PhysicsManagerAttributes::setTimestep(double timestep) {
  setDouble("timestep", timestep);
}
double PhysicsManagerAttributes::getTimestep() const {
  return getDouble("timestep");
}

void PhysicsManagerAttributes::setMaxSubsteps(int maxSubsteps) {
  setInt("maxSubsteps", maxSubsteps);
}
int PhysicsManagerAttributes::getMaxSubsteps() const {
  return getInt("maxSubsteps");
}
}  // namespace assets
}  // namespace esp
