#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Utility/Arguments.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Json.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/DebugTools/FrameProfiler.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/Platform/GlfwApplication.h>

#include "esp/sensor/CameraSensor.h"
#include "esp/sim/BatchReplayRenderer.h"
#include "esp/sim/ClassicReplayRenderer.h"

namespace {

namespace Cr = Corrade;
namespace Mn = Magnum;
using namespace Cr::Containers::Literals;
using namespace Mn::Math::Literals;

class Replayer : public Mn::Platform::Application {
 public:
  explicit Replayer(const Arguments& arguments);

 private:
  void drawEvent() override;
  void mousePressEvent(MouseEvent& event) override;

  esp::logging::LoggingContext loggingContext_;

  /* jsonKeyframes[jsonFileKeyframeOffsets[i]] to
     jsonKeyframes[jsonFileKeyframeOffsets[i+1]] contains keyframes for file
     i, the strings are owned by a corresponding instance in jsonFiles. */
  Cr::Containers::Array<Cr::Utility::Json> jsonFiles_;
  Cr::Containers::Array<Cr::Containers::StringView> keyframes_;
  Cr::Containers::Array<std::size_t> fileKeyframeOffsets_;

  Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer> replayRenderer_;

  std::size_t maxFrameCount_ = 0;
  std::size_t frameIndex_ = 0;
  bool paused_ = false;
  bool once_ = false;

  Mn::DebugTools::FrameProfilerGL profiler_;
};

Replayer::Replayer(const Arguments& arguments)
    : Mn::Platform::Application{arguments, Mn::NoCreate} {
  Cr::Utility::Arguments args;
  args.addArrayArgument("json")
      .setHelp("json", "gfx-replay JSON file(s) to use")
      .addArrayOption('P', "preload")
      .setHelp("preload", "composite file(s) to preload", "file.glb")
      .addOption("size", "512 384")
      .setHelp("size", "environment size", "\"X Y\"")
      .addOption("max-light-count", "0")
      .setHelp("max-light-count", "max light count used per scene")
      .addBooleanOption("classic")
      .setHelp("classic", "use the classic renderer")
      .addBooleanOption("profile")
      .setHelp("profile", "print frame profiling info to the console")
      .addBooleanOption("once")
      .setHelp("once",
               "run through the replay just once instead of repeating forever, "
               "useful for benchmarking the renderer without the gfx-replay "
               "parsing overhead")
      .addSkippedPrefix("magnum", "engine-specific options")
      .setGlobalHelp(R"(
Plays back one or more gfx-replay JSON files, optionally using a set of
composite files to render from instead of loading individual meshes.

When more JSON files are supplied, they're organized in a grid. Use --size to
control how large is each grid cell. By default each individual file is
repeated forever, use --once to run each just once. A mouse click pauses /
unpauses the playback.

By default the batch renderer is used, however it implicitly loads individual
files from the filesystem as referenced by the gfx-replay JSON. If you pass one
or more composite files via the -P / --preload option, it'll prefer them if
they contain given referenced file, and will fall back to loading from the
filesystem if not, printing a warning. The --classic option switches to the
classic renderer and resource management, --preload is ignored in that case.

Default rendering is flat-shaded. Set --max-light-count to a value higher than
0 to consume also light information from the gfx-replay JSON files. Meshes with
materials marked as Flat will be rendered flat-shaded even with lights present.

For simple profiling, the --profile option will print GPU, CPU and total frame
time to the console. It includes the JSON parsing overhead as well, to
benchmark just the renderer itself use --once and wait until all animations
settle down.
)"_s.trimmed())
      .parse(arguments.argc, arguments.argv);

  once_ = args.isSet("once");
  const std::size_t fileCount = args.arrayValueCount("json");

  create(Configuration{}
             .setSize(args.value<Mn::Vector2i>("size") *
                      esp::sim::AbstractReplayRenderer::environmentGridSize(
                          fileCount))
             .setTitle("Replayer"));

  arrayReserve(jsonFiles_, fileCount);
  fileKeyframeOffsets_ =
      Cr::Containers::Array<std::size_t>{Cr::ValueInit, fileCount + 1};
  for (std::size_t i = 0; i != fileCount; ++i) {
    const auto filename =
        args.arrayValue<Cr::Containers::StringView>("json", i);

    /* Just tokenize the file and process the bare minimum needed to be able to
       go through the keyframe list */
    Cr::Containers::Optional<Cr::Utility::Json> jsonFile =
        Cr::Utility::Json::fromFile(filename);
    Cr::Containers::Optional<Cr::Utility::JsonObjectView> root;
    if (!jsonFile || !(root = jsonFile->parseObject(jsonFile->root())))
      Mn::Fatal{} << "Can't parse" << filename;
    const Cr::Utility::JsonToken* jsonKeyframes;
    if (!(jsonKeyframes = (*root).find("keyframes")) ||
        !jsonFile->parseArray(*jsonKeyframes))
      Mn::Fatal{} << "No keyframes array in" << filename;

    /* Put raw keyframe data into the array */
    for (const Cr::Utility::JsonToken& keyframe : jsonKeyframes->asArray())
      arrayAppend(keyframes_, keyframe.data());

    /* Save the Utility::Json instance, remember how many keyframes was added
       from this file */
    if (fileKeyframeOffsets_[i] == keyframes_.size())
      Mn::Fatal{} << "No keyframes found in" << filename;
    arrayAppend(jsonFiles_, *std::move(jsonFile));
    fileKeyframeOffsets_[i + 1] = keyframes_.size();

    maxFrameCount_ = Mn::Math::max(
        maxFrameCount_, fileKeyframeOffsets_[i + 1] - fileKeyframeOffsets_[i]);
  }

  Mn::Debug{} << "Playing" << maxFrameCount_ << "frames from" << fileCount
              << "files";
  /* If we're repeating forever there's no max frame count after which the
     redrawing should stop */
  if (!once_)
    maxFrameCount_ = ~std::size_t{};

  std::string sensorName = "my_rgb";
  std::string userPrefix = "sensor_";

  std::vector<esp::sensor::SensorSpec::ptr> sensorSpecifications;
  {
    auto pinholeCameraSpec = esp::sensor::CameraSensorSpec::create();
    pinholeCameraSpec->sensorSubType = esp::sensor::SensorSubType::Pinhole;
    pinholeCameraSpec->sensorType = esp::sensor::SensorType::Color;
    pinholeCameraSpec->position = {0.0f, 0.f, 0.0f};
    /* Account for DPI scaling, i.e. take the actual framebuffer resolution
       divided by the environment grid size instead of the resolution specified
       in the arguments */
    pinholeCameraSpec->resolution = Mn::EigenIntegration::cast<esp::vec2i>(
        (framebufferSize() /
         esp::sim::AbstractReplayRenderer::environmentGridSize(fileCount))
            .flipped());
    pinholeCameraSpec->uuid = sensorName;
    sensorSpecifications = {pinholeCameraSpec};
  }

  esp::sim::ReplayRendererConfiguration rendererConfig;
  rendererConfig.sensorSpecifications = std::move(sensorSpecifications);
  rendererConfig.numEnvironments = fileCount;
  rendererConfig.standalone = false;
  esp::gfx_batch::RendererConfiguration batchRendererConfig;
  batchRendererConfig.setMaxLightCount(
      args.value<Mn::UnsignedInt>("max-light-count"));
  if (args.isSet("classic"))
    replayRenderer_.emplace<esp::sim::ClassicReplayRenderer>(rendererConfig);
  else
    replayRenderer_.emplace<esp::sim::BatchReplayRenderer>(
        rendererConfig, std::move(batchRendererConfig));

  for (std::size_t i = 0, iMax = args.arrayValueCount("preload"); i != iMax;
       ++i)
    replayRenderer_->preloadFile(args.arrayValue("preload", i));

  // We're going to be lazy and only set up one buffer/imageView per env, on the
  // assumption that there's only one sensor.
  CORRADE_INTERNAL_ASSERT(rendererConfig.sensorSpecifications.size() == 1);

  profiler_ = Mn::DebugTools::FrameProfilerGL{
      Mn::DebugTools::FrameProfilerGL::Value::FrameTime |
          Mn::DebugTools::FrameProfilerGL::Value::CpuDuration |
          Mn::DebugTools::FrameProfilerGL::Value::GpuDuration,
      50};
  if (!args.isSet("profile"))
    profiler_.disable();
}

void Replayer::drawEvent() {
  Mn::GL::defaultFramebuffer.clear(Mn::GL::FramebufferClear::Color |
                                   Mn::GL::FramebufferClear::Depth);

  profiler_.beginFrame();

  if (!paused_) {
    for (std::size_t envIndex = 0; envIndex < jsonFiles_.size(); ++envIndex) {
      const Cr::Containers::ArrayView<const Cr::Containers::StringView>
          keyframesForEnvironment =
              keyframes_.slice(fileKeyframeOffsets_[envIndex],
                               fileKeyframeOffsets_[envIndex + 1]);
      /* Beware, we can't set arbitrary keyframes, as they are usually
         generated. We must set them exactly in order, or clear and start
         over. */
      const std::size_t environmentFrameIndex =
          once_ ? frameIndex_ : frameIndex_ % keyframesForEnvironment.size();
      if (frameIndex_ != 0 && environmentFrameIndex == 0)
        replayRenderer_->clearEnvironment(envIndex);
      if (environmentFrameIndex < keyframesForEnvironment.size()) {
        replayRenderer_->setEnvironmentKeyframeUnwrapped(
            envIndex, keyframesForEnvironment[environmentFrameIndex]);
      }

      const auto eyePos = Mn::Vector3(-1.5f, 1.75f - (float)envIndex * 0.1f,
                                      -0.5f + (float)envIndex * 0.5f);
      Mn::Matrix4 transform = Mn::Matrix4::lookAt(
          eyePos,
          eyePos + Mn::Vector3(2.f - (float)environmentFrameIndex * 0.002f,
                               -0.5f, 1.f),
          {0.f, 1.f, 0.f});
      replayRenderer_->setSensorTransform(envIndex, "my_rgb", transform);
    }

    ++frameIndex_;
  }

  replayRenderer_->render(Mn::GL::defaultFramebuffer);

  profiler_.endFrame();
  profiler_.printStatistics(10);

  /* Stop redrawing if we're playing just once once and we reached all frames.
     Also don't waste the CPU if we're paused and the profiler isn't
     enabled. */
  if ((frameIndex_ < maxFrameCount_ && !paused_) || profiler_.isEnabled())
    redraw();
  swapBuffers();
}

void Replayer::mousePressEvent(MouseEvent& event) {
  if (event.button() == MouseEvent::Button::Left)
    paused_ ^= true;
  else
    return;

  event.setAccepted();
  redraw();
}

}  // namespace

MAGNUM_APPLICATION_MAIN(Replayer)
