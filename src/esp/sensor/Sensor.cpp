// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Sensor.h"

#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>

namespace esp {
namespace sensor {

Sensor::Sensor(scene::SceneNode& node, SensorSpec::ptr spec)
    : Magnum::SceneGraph::AbstractFeature3D{node},
      spec_(spec),
      framebufferSize_(0, 0),
      colorBuffer_(),
      depthBuffer_(),
      objectIdBuffer_(),
      depthRenderbuffer_(),
      framebuffer_(NoCreate) {
  node.setType(scene::SceneNodeType::SENSOR);
  if (spec_ == nullptr) {
    LOG(ERROR) << "Cannot initialize sensor. The specification is null.";
  }
  ASSERT(spec_ != nullptr);

  framebufferSize_ = Magnum::Vector2i{spec_->resolution};
  colorBuffer_.setStorage(GL::RenderbufferFormat::SRGB8Alpha8,
                          framebufferSize_);
  depthBuffer_.setStorage(GL::RenderbufferFormat::R32F, framebufferSize_);
  objectIdBuffer_.setStorage(GL::RenderbufferFormat::R32UI, framebufferSize_);
  depthRenderbuffer_.setStorage(GL::RenderbufferFormat::Depth24Stencil8,
                                framebufferSize_);
  framebuffer_ = GL::Framebuffer{{{}, framebufferSize_}};
  framebuffer_
      .attachRenderbuffer(GL::Framebuffer::ColorAttachment{0}, colorBuffer_)
      .attachRenderbuffer(GL::Framebuffer::ColorAttachment{1}, depthBuffer_)
      .attachRenderbuffer(GL::Framebuffer::ColorAttachment{2}, objectIdBuffer_)
      .attachRenderbuffer(GL::Framebuffer::BufferAttachment::Depth,
                          depthRenderbuffer_)
      .mapForDraw({{0, GL::Framebuffer::ColorAttachment{0}},
                   {1, GL::Framebuffer::ColorAttachment{1}},
                   {2, GL::Framebuffer::ColorAttachment{2}}});
  CORRADE_INTERNAL_ASSERT(
      framebuffer_.checkStatus(GL::FramebufferTarget::Draw) ==
      GL::Framebuffer::Status::Complete);

  setTransformationFromSpec();
}

Observation Sensor::getObservation() {
  // TODO fill out observation
  Observation obs{};
  return obs;
}

void SensorSuite::add(Sensor::ptr sensor) {
  const std::string uuid = sensor->specification()->uuid;
  sensors_[uuid] = sensor;
}

Sensor::ptr SensorSuite::get(const std::string& uuid) const {
  return (sensors_.at(uuid));
}

void SensorSuite::clear() {
  sensors_.clear();
}

void Sensor::setTransformationFromSpec() {
  if (spec_ == nullptr) {
    LOG(ERROR) << "Cannot initialize sensor. the specification is null.";
    return;
  }

  node().resetTransformation();

  node().translate(Magnum::Vector3(spec_->position));
  node().rotateX(Magnum::Rad(spec_->orientation[0]));
  node().rotateY(Magnum::Rad(spec_->orientation[1]));
  node().rotateZ(Magnum::Rad(spec_->orientation[2]));
}

void Sensor::renderEnter() {
  framebuffer_.bind();
  framebuffer_.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);
  framebuffer_.clearColor(1, Vector4{});
  framebuffer_.clearColor(2, Vector4ui{});
  framebuffer_.bind();
}

void Sensor::renderExit() {}

void Sensor::readFrameRgba(uint8_t* ptr) {
  framebuffer_.mapForRead(GL::Framebuffer::ColorAttachment{0});
  Image2D rgbaImage = framebuffer_.read(
      Range2Di::fromSize({0, 0}, framebufferSize_), {PixelFormat::RGBA8Unorm});
  uint8_t* src_ptr = rgbaImage.data<uint8_t>();
  std::memcpy(ptr, src_ptr,
              framebufferSize_[0] * framebufferSize_[1] * 4 * sizeof(uint8_t));
}

void Sensor::readFrameDepth(float* ptr) {
  framebuffer_.mapForRead(GL::Framebuffer::ColorAttachment{1});
  Image2D depthImage = framebuffer_.read(
      Range2Di::fromSize({0, 0}, framebufferSize_), {PixelFormat::R32F});
  float* src_ptr = depthImage.data<float>();
  std::memcpy(ptr, src_ptr,
              framebufferSize_[0] * framebufferSize_[1] * sizeof(float));
}

void Sensor::readFrameObjectId(uint32_t* ptr) {
  framebuffer_.mapForRead(GL::Framebuffer::ColorAttachment{2});
  Image2D objectImage = framebuffer_.read(
      Range2Di::fromSize({0, 0}, framebufferSize_), {PixelFormat::R32UI});
  uint32_t* src_ptr = objectImage.data<uint32_t>();
  std::memcpy(ptr, src_ptr,
              framebufferSize_[0] * framebufferSize_[1] * sizeof(uint32_t));
}

gltensor::GLTensorParam::ptr Sensor::glTensorParam() const {
  auto param = std::make_shared<gltensor::GLTensorParam>();

  param->height_ = framebufferSize_[0];
  param->width_ = framebufferSize_[1];

  param->target_ = GL_TEXTURE_2D;

  switch (spec_->sensorType) {
    case SensorType::COLOR:
      param->format_ = GL_RGBA;
      param->image_ = colorBuffer_.id();
      break;

    case SensorType::DEPTH:
      param->format_ = GL_R32F;
      param->image_ = depthBuffer_.id();
      break;

    case SensorType::SEMANTIC:
      param->format_ = GL_R32UI;
      param->image_ = objectIdBuffer_.id();
      break;

    default:
      throw std::runtime_error(
          "GLTensor only suppots color, depth, and semantic sensors");
  }

  return param;
}

bool operator==(const SensorSpec& a, const SensorSpec& b) {
  return a.uuid == b.uuid && a.sensorType == b.sensorType &&
         a.sensorSubtype == b.sensorSubtype && a.parameters == b.parameters &&
         a.position == b.position && a.orientation == b.orientation &&
         a.resolution == b.resolution && a.channels == b.channels &&
         a.encoding == b.encoding && a.observationSpace == b.observationSpace;
}
bool operator!=(const SensorSpec& a, const SensorSpec& b) {
  return !(a == b);
}

}  // namespace sensor
}  // namespace esp
