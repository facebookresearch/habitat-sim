// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/StridedArrayView.h>
#include <Corrade/Containers/Triple.h>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/Utility/Format.h>
#include <Corrade/Utility/Path.h>
#include <Corrade/Utility/String.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/DebugTools/CompareImage.h>
#include <Magnum/GL/OpenGLTester.h> /* just for MAGNUM_VERIFY_NO_GL_ERROR() */
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/SceneData.h>

#include "esp/gfx_batch/RendererStandalone.h"

#include "configure.h"

namespace {

namespace Cr = Corrade;
namespace Mn = Magnum;
using namespace Cr::Containers::Literals;
using namespace Mn::Math::Literals;

struct FloorplannerProcessingTest : Cr::TestSuite::Tester {
  explicit FloorplannerProcessingTest();

  void sceneListSortedUnique();

  void original();      /* 1 to 210 */
  void json();          /* 211 to 419 */
  void jsonComposite(); /* 420 to 629 */
};

constexpr Mn::Vector2i BaseRenderSize{1024, 1024};

/* Having all images the same size would mean different comparison thresholds
   get applied for the same feature across diffently sized scenes, which is not
   wanted. Instead, each image is rendered with a different FoV to a different
   size. */
const struct {
  Cr::TestSuite::TestCaseDescriptionSourceLocation name;
  Mn::Vector2 translation;
  Mn::Vector2 scale;
  Mn::Float maxThreshold, meanThreshold;
  std::size_t expectedNodeCount, expectedDrawCount, expectedBatchCount;
} Data[]{
  /* The names should be sorted and unique. Checked in
     sceneListSortedUnique(). */
  {"102343992", {-10.0f, 0.0f}, {1.25f, 1.5f},
    256.0f, 2.0f, 0, 0, 2},
  {"102344022", {9.0f, -4.0f}, {0.875f, 0.75f},
    256.0f, 2.0f, 0, 0, 2},
  {"102344049", {-2.0f, -7.0f}, {0.875f, 0.75f},
    256.0f, 2.0f, 0, 0, 2},
  {"102344094", {7.0f, -2.0f}, {0.375f, 0.375f},
    256.0f, 2.0f, 0, 0, 2},

  {"102344115", {4.0f, -3.0f}, {0.25f, 0.375f},
    256.0f, 2.0f, 0, 0, 2},
  {"102344193", {2.0f, -5.0f}, {0.625f, 0.375f},
    256.0f, 2.0f, 0, 0, 2},
  {"102344250", {9.0f, -8.0f}, {0.75f, 0.625f},
    256.0f, 2.0f, 0, 0, 2},
  {"102344280", {1.0f, 3.0f}, {0.875f, 0.75f},
    256.0f, 2.0f, 0, 0, 2},

  {"102344307", {9.0f, -9.0f}, {0.75f, 0.625f},
    256.0f, 2.0f, 0, 0, 2},
  {"102344328", {-5.0f, -6.0f}, {1.625f, 0.625f},
    256.0f, 2.0f, 0, 0, 2},
  {"102344349", {8.0f, -6.0f}, {0.875f, 0.625f},
    256.0f, 2.0f, 0, 0, 2},
  {"102344403", {-1.0f, 4.0f}, {1.875f, 1.75f},
    256.0f, 2.0f, 0, 0, 2},

  {"102344439", {9.0f, 6.0f}, {1.125f, 1.0f},
    256.0f, 2.0f, 0, 0, 2},
  {"102344457", {-9.0f, -14.0f}, {0.75f, 0.625f},
    256.0f, 2.0f, 0, 0, 2},
  {"102344469", {20.0f, -2.0f}, {1.375f, 0.375f},
    256.0f, 2.0f, 0, 0, 2},
  {"102344529", {2.0f, -2.0f}, {0.875f, 0.625f},
    256.0f, 2.0f, 0, 0, 2},

  // TODO add images and translation/scale for the rest
  {"102815835", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102815859", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102815859_169535055", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102816009", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"102816036", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102816051", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102816066", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102816114", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"102816150", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102816201", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102816216", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102816600", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"102816615", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102816627", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102816729", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102816756", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"102816786", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102816852", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102817053", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102817119", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"102817140", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102817161", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"102817200", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997403_171030405", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"103997424_171030444", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997445_171030492", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997460_171030507", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997478_171030525", {-5.0f, 0.0f}, {1.125f, 0.875f},
    256.0f, 2.1f, 4517, 4328, 8}, // TODO median higher because window z fighting

  {"103997478_171030528", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997541_171030615", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997562_171030642", {8.0f, -11.0f}, {1.375f, 0.75f},
    256.0f, 4.0f, 3783, 3532, 8},
  {"103997586_171030666", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"103997586_171030669", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997613_171030702", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997643_171030747", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997718_171030855", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"103997730_171030885", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997781_171030978", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997799_171031002", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997865_171031116", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"103997895_171031182", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997919_171031233", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997940_171031257", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"103997970_171031287", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"103997994_171031320", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104348010_171512832", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104348028_171512877", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104348037_171512898", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"104348064_171512940", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104348082_171512994", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104348103_171513021", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104348133_171513054", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"104348160_171513093", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104348181_171513120", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104348202_171513150", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104348253_171513237", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"104348289_171513294", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104348328_171513363", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104348361_171513414", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104348394_171513453", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"104348463_171513588", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104348478_171513603", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104348511_171513654", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862345_172226274", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"104862369_172226304", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862384_172226319", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862396_172226349", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862417_172226382", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"104862474_172226496", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862501_172226556", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862513_172226580", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862534_172226625", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"104862558_172226664", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862573_172226682", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862579_172226694", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862609_172226751", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"104862621_172226772", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862639_172226823", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862660_172226844", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862669_172226853", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"104862681_172226874", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862687_172226883", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"104862726_172226952", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515151_173104068", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"105515160_173104077", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  // TODO this one is a three-story building, needs the roof and upper floors removed (thus rendering three times? or adding it three times next to itself into a single image, each time with a different clipping plane?)
  // {"105515175_173104107", {0.0f, 6.0f}, {2.125f, 2.375f},
    // 256.0f, 2.0f, 0, 0, 2},
  {"105515184_173104128", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515211_173104173", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"105515211_173104179", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515211_173104185", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515235_173104215", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515265_173104248", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"105515286_173104287", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515301_173104305", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515307_173104317", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515322_173104332", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"105515337_173104347", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515364_173104374", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515379_173104395", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515403_173104449", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"105515430_173104494", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515448_173104512", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515490_173104566", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515505_173104584", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"105515523_173104614", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"105515541_173104641", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106365897_174225969", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106365897_174225972", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"106366104_174226320", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106366104_174226329", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106366104_174226332", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106366134_174226362", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"106366173_174226431", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106366185_174226443", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106366233_174226506", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106366248_174226527", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"106366263_174226557", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106366293_174226605", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106366302_174226617", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106366323_174226647", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"106366335_174226662", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106366353_174226695", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106366371_174226743", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106366386_174226770", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"106366410_174226806", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106366413_174226809", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106366434_174226881", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106878840_174886947", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"106878858_174886965", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106878867_174886977", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106878915_174887025", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106878945_174887058", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"106878960_174887073", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106878975_174887088", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106879005_174887124", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106879023_174887148", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"106879044_174887172", {8.0f, -8.0f}, {0.5f, 0.5f},
    256.0f, 2.0f, 0, 0, 2},
  {"106879080_174887211", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"106879104_174887235", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107733912_175999623", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"107733945_175999674", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107733960_175999701", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107733978_175999737", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107734017_175999794", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"107734056_175999839", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  // TODO also roof
  // {"107734080_175999881", {0.0f, 0.0f}, {3.0f, 3.0f},
    // 256.0f, 2.0f, 0, 0, 2},
  {"107734110_175999914", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107734110_175999917", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"107734119_175999932", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107734119_175999935", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107734119_175999938", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107734146_175999971", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"107734158_175999998", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107734176_176000019", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107734188_176000034", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107734227_176000091", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"107734254_176000121", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107734287_176000160", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107734338_176000244", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107734410_176000355", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"107734449_176000403", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"107734479_176000442", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108294417_176709879", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108294465_176709960", {5.0f, -5.0f}, {0.25f, 0.375f},
    256.0f, 2.0f, 0, 0, 2},

  {"108294492_176709993", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108294537_176710050", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108294558_176710095", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108294573_176710113", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"108294600_176710152", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108294624_176710203", {-1.0f, -4.0f}, {0.75f, 0.875f},
    256.0f, 2.0f, 0, 0, 2},
  {"108294684_176710278", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108294765_176710386", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"108294783_176710410", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108294798_176710428", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108294816_176710461", {9.0f, -5.0f}, {1.5f, 0.5f},
    256.0f, 2.0f, 0, 0, 2},
  {"108294846_176710506", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"108294870_176710551", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108294897_176710602", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108294915_176710620", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108294927_176710650", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"108294939_176710668", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108736611_177263226", {4.0f, -9.0f}, {0.75f, 0.875f},
    256.0f, 2.0f, 0, 0, 2},
  {"108736635_177263256", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108736656_177263304", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"108736677_177263328", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108736689_177263340", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108736704_177263361", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108736722_177263382", {7.0f, -1.0f}, {0.5f, 0.375f},
    256.0f, 2.0f, 0, 0, 2},

  {"108736737_177263406", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108736779_177263484", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108736800_177263517", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108736824_177263559", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},

  {"108736851_177263586", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108736872_177263607", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
  {"108736884_177263634", {0.0f, 0.0f}, {1.0f, 1.0f},
    256.0f, 2.0f, 0, 0, 1},
};

FloorplannerProcessingTest::FloorplannerProcessingTest() {
  addTests({&FloorplannerProcessingTest::sceneListSortedUnique});

  addInstancedTests({&FloorplannerProcessingTest::original},
    Cr::Containers::arraySize(Data));
  addInstancedTests({&FloorplannerProcessingTest::json},
    Cr::Containers::arraySize(Data));
  addInstancedTests({&FloorplannerProcessingTest::jsonComposite},
    Cr::Containers::arraySize(Data));
}

void FloorplannerProcessingTest::sceneListSortedUnique() {
  Cr::Containers::StringView prev;

  CORRADE_COMPARE_AS(Cr::Containers::arraySize(Data), 211,
    Cr::TestSuite::Compare::LessOrEqual);
  if(Cr::Containers::arraySize(Data) < 211)
    CORRADE_WARN("Only" << Cr::Containers::arraySize(Data) << "scenes tested out of 211");

  for(std::size_t i = 0; i != Cr::Containers::arraySize(Data); ++i) {
    CORRADE_COMPARE_AS(Data[i].name, prev, Cr::TestSuite::Compare::Greater);
    prev = Data[i].name;

    /* The translation should have no fractional part */
    CORRADE_COMPARE(Mn::Math::round(Data[i].translation), Data[i].translation);

    /* The scale should be 1/8ths at least */
    CORRADE_COMPARE_AS(Mn::Int(Data[i].scale.x()*1000), 125,
      Cr::TestSuite::Compare::Divisible);
    CORRADE_COMPARE_AS(Mn::Int(Data[i].scale.y()*1000), 125,
      Cr::TestSuite::Compare::Divisible);
  }
}

void FloorplannerProcessingTest::original() {
  auto&& data = Data[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  const Mn::Vector2i renderSize = BaseRenderSize*data.scale;

  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount(renderSize, {1, 1}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  const Cr::Containers::String filename = Cr::Utility::Path::join(DATA_DIR, Cr::Utility::format("fp/scenes/original/{}.glb", Cr::Containers::StringView{data.name}));
  if(!Cr::Utility::Path::exists(filename))
    CORRADE_SKIP(filename << "doesn't exist");

  /* GenerateMipmap to avoid signficant aliasing and thus potentially a lot of
     comparison differences */
  CORRADE_VERIFY(renderer.addFile(filename, esp::gfx_batch::RendererFileFlag::Whole|esp::gfx_batch::RendererFileFlag::GenerateMipmap));
  CORRADE_COMPARE(renderer.addNodeHierarchy(0, filename, {}), 0);

  /* Perspective projection to be able to see walls also a bit */
  renderer.camera(0) =
    Mn::Matrix4::perspectiveProjection(35.0_degf*data.scale.x(), Mn::Vector2{data.scale}.aspectRatio(), 1.0f, 100.0f)*
    Mn::Matrix4::translation({data.translation, -50.0f})*
    Mn::Matrix4::rotationX(90.0_degf);

  renderer.draw();
  Mn::Image2D color = renderer.colorImage();
  MAGNUM_VERIFY_NO_GL_ERROR();

  CORRADE_COMPARE_WITH(color,
      Cr::Utility::Path::join({TEST_ASSETS,
                              "screenshots/floorplanner/", data.name + ".png"_s}),
      (Mn::DebugTools::CompareImageToFile{}));

  /* Verify that there isn't anything out of view, i.e. that a strip along the
     edges has just the background color */
  constexpr std::size_t EdgeSize = 10;
  for(auto edge: {
    color.pixels<Mn::Color4ub>().prefix(EdgeSize), /* bottom */
    color.pixels<Mn::Color4ub>().flipped<0>().prefix(EdgeSize), /* top */
    color.pixels<Mn::Color4ub>().transposed<0, 1>().prefix(EdgeSize), /* left */
    color.pixels<Mn::Color4ub>().transposed<0, 1>().flipped<0>().prefix(EdgeSize), /* right */
  }) {
    for(auto row: edge)
      for(auto pixel: row)
        CORRADE_FAIL_IF(pixel != 0x1f1f1fff_rgba,
          "Render is out of view");
  }
}

void FloorplannerProcessingTest::json() {
  auto&& data = Data[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  const Mn::Vector2i renderSize = BaseRenderSize*data.scale;

  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount(renderSize, {1, 1}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  /* Skip if the JSON isn't available (i.e., the dataset isn't present) */
  const Cr::Containers::String basePath = Cr::Utility::Path::join(DATA_DIR, "fphab/scenes/");
  const Cr::Containers::String filename = Cr::Utility::Path::join(basePath, data.name + ".scene_instance.json"_s);
  if(!Cr::Utility::Path::exists(filename))
    CORRADE_SKIP(filename << "doesn't exist");

  /* Parse the JSON for external filenames + transformations */
  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> manager;
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer = manager.loadAndInstantiate(HABITAT_IMPORTER_PLUGIN);
  CORRADE_VERIFY(importer);
  CORRADE_VERIFY(importer->openFile(filename));
  Cr::Containers::Optional<Mn::Trade::SceneData> scene = importer->scene(0);
  CORRADE_VERIFY(scene);

  /* Query the external file field */
  const Mn::Trade::SceneField sceneFieldExternalFile = importer->sceneFieldForName("externalFile");
  CORRADE_VERIFY(sceneFieldExternalFile != Mn::Trade::SceneField{});
  const Cr::Containers::Optional<Mn::UnsignedInt> externalFileFieldId = scene->findFieldId(sceneFieldExternalFile);
  CORRADE_VERIFY(externalFileFieldId);

  /* Gather transformations for all objects. Objects that don't have it are
     kept at the default identity transformation. */
  Cr::Containers::Array<Mn::Matrix4> transformations{Cr::ValueInit, std::size_t(importer->objectCount())};
  for(Cr::Containers::Pair<Mn::UnsignedInt, Mn::Matrix4> transformation: scene->transformations3DAsArray()) {
    transformations[transformation.first()] = transformation.second();
  }

  /* Go through all referenced external files and add them with their
     transformation */
  {
    CORRADE_COMPARE(scene->mappingType(), Mn::Trade::SceneMappingType::UnsignedInt);
    Cr::Containers::StridedArrayView1D<const Mn::UnsignedInt> mappings = scene->mapping<Mn::UnsignedInt>(*externalFileFieldId);
    Cr::Containers::StringIterable files = scene->fieldStrings(*externalFileFieldId);
    for(std::size_t i = 0; i != scene->fieldSize(*externalFileFieldId); ++i) {
      const Cr::Containers::String filename = Cr::Utility::Path::join(basePath, files[i]);

      /* Add the file only if it's not already present (such as the same chair
         used multiple times in different locations) */
      if(!renderer.hasNodeHierarchy(filename))
        /* GenerateMipmap to avoid signficant aliasing and thus potentially a
           lot of comparison differences */
        CORRADE_VERIFY(renderer.addFile(filename, esp::gfx_batch::RendererFileFlag::Whole|esp::gfx_batch::RendererFileFlag::GenerateMipmap));
      renderer.addNodeHierarchy(0, filename, transformations[mappings[i]]);
    }
  }

  /* Perspective projection to be able to see walls also a bit */
  renderer.camera(0) =
    Mn::Matrix4::perspectiveProjection(35.0_degf*data.scale.x(), Mn::Vector2{data.scale}.aspectRatio(), 1.0f, 100.0f)*
    Mn::Matrix4::translation({data.translation, -50.0f})*
    Mn::Matrix4::rotationX(90.0_degf);

  renderer.draw();
  Mn::Image2D color = renderer.colorImage();
  MAGNUM_VERIFY_NO_GL_ERROR();

  // TODO fix these
  {
    CORRADE_EXPECT_FAIL("The JSON-produced datasets are currently significantly different from the original files.");
    CORRADE_COMPARE_WITH(color,
        Cr::Utility::Path::join({TEST_ASSETS,
                                "screenshots/floorplanner/", data.name + ".png"_s}),
        (Mn::DebugTools::CompareImageToFile{}));
  } {
    /* They should be not too different, though */
    CORRADE_COMPARE_WITH(color,
        Cr::Utility::Path::join({TEST_ASSETS,
                                "screenshots/floorplanner/", data.name + ".png"_s}),
        (Mn::DebugTools::CompareImageToFile{256.0f, 20.0f}));
  }

  CORRADE_COMPARE_WITH(color,
      Cr::Utility::Path::join({TEST_ASSETS,
                              "screenshots/floorplanner-json/", data.name + ".png"_s}),
      (Mn::DebugTools::CompareImageToFile{}));
}

void FloorplannerProcessingTest::jsonComposite() {
  auto&& data = Data[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  const Mn::Vector2i renderSize = BaseRenderSize*data.scale;

  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount(renderSize, {1, 1}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  /* Skip if the JSON isn't available (i.e., the dataset isn't present) */
  const Cr::Containers::String basePath = Cr::Utility::Path::join(DATA_DIR, "fphab/scenes/");
  const Cr::Containers::String filename = Cr::Utility::Path::join(basePath, data.name + ".scene_instance.json"_s);
  if(!Cr::Utility::Path::exists(filename))
    CORRADE_SKIP(filename << "doesn't exist");

  /* Skip if the composite files aren't available */
  // TODO ugh something stable instead
  const Cr::Containers::String compositeFilename = Cr::Utility::Path::join(DATA_DIR, "floorplanner-full/composite-1000-split.gltf");
  if(!Cr::Utility::Path::exists(compositeFilename))
    CORRADE_SKIP(compositeFilename << "doesn't exist");
  // const Cr::Containers::String compositeFurnitureFilename = Cr::Utility::Path::join(DATA_DIR, "composite-fp-dedup2/furniture-subset-ca7f00d.gltf");
  // if(!Cr::Utility::Path::exists(compositeFurnitureFilename))
  //   CORRADE_SKIP(compositeFurnitureFilename << "doesn't exist");

  /* Load the composites */
  // TODO have them contain mipmaps, otherwise it'll be aliased hell
  CORRADE_VERIFY(renderer.addFile(compositeFilename));
  // CORRADE_VERIFY(renderer.addFile(compositeFurnitureFilename));

  /* Parse the JSON for external filenames + transformations */
  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> manager;
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer = manager.loadAndInstantiate(HABITAT_IMPORTER_PLUGIN);
  CORRADE_VERIFY(importer);
  CORRADE_VERIFY(importer->openFile(filename));
  Cr::Containers::Optional<Mn::Trade::SceneData> scene = importer->scene(0);
  CORRADE_VERIFY(scene);

  /* Query the external file field */
  const Mn::Trade::SceneField sceneFieldExternalFile = importer->sceneFieldForName("externalFile");
  CORRADE_VERIFY(sceneFieldExternalFile != Mn::Trade::SceneField{});
  const Cr::Containers::Optional<Mn::UnsignedInt> externalFileFieldId = scene->findFieldId(sceneFieldExternalFile);
  CORRADE_VERIFY(externalFileFieldId);

  /* Gather transformations for all objects. Objects that don't have it are
     kept at the default identity transformation. */
  Cr::Containers::Array<Mn::Matrix4> transformations{Cr::ValueInit, std::size_t(importer->objectCount())};
  for(Cr::Containers::Pair<Mn::UnsignedInt, Mn::Matrix4> transformation: scene->transformations3DAsArray()) {
    transformations[transformation.first()] = transformation.second();
  }

  /* Go through all referenced external files and add them with their
     transformation */
  {
    CORRADE_COMPARE(scene->mappingType(), Mn::Trade::SceneMappingType::UnsignedInt);
    Cr::Containers::StridedArrayView1D<const Mn::UnsignedInt> mappings = scene->mapping<Mn::UnsignedInt>(*externalFileFieldId);
    Cr::Containers::StringIterable files = scene->fieldStrings(*externalFileFieldId);
    for(std::size_t i = 0; i != scene->fieldSize(*externalFileFieldId); ++i) {
      if(files[i].hasPrefix("../objects/decomposed")) {
        CORRADE_WARN("Skipping" << files[i] << "not currently present in the composite due to bugs in source data");
        continue;
      }

      /* Add the file only if it's not already present (such as the same chair
         used multiple times in different locations) */
      CORRADE_FAIL_IF(!renderer.hasNodeHierarchy(files[i]),
        files[i] << "not present in the composite");

      renderer.addNodeHierarchy(0, files[i], transformations[mappings[i]]);
    }
  }

  /* Perspective projection to be able to see walls also a bit */
  renderer.camera(0) =
    Mn::Matrix4::perspectiveProjection(35.0_degf*data.scale.x(), Mn::Vector2{data.scale}.aspectRatio(), 1.0f, 100.0f)*
    Mn::Matrix4::translation({data.translation, -50.0f})*
    Mn::Matrix4::rotationX(90.0_degf);

  renderer.draw();
  Mn::Image2D color = renderer.colorImage();
  MAGNUM_VERIFY_NO_GL_ERROR();

  // TODO compare against the originals once the process is fixed
  CORRADE_COMPARE_WITH(color,
      Cr::Utility::Path::join({TEST_ASSETS,
                              "screenshots/floorplanner-json/", data.name + ".png"_s}),
      (Mn::DebugTools::CompareImageToFile{data.maxThreshold, data.meanThreshold}));

  esp::gfx_batch::SceneStats stats = renderer.sceneStats(0);
  // TODO use three separate comparisons with .fallthrough() instead once
  //  implemented in TestSuite
  CORRADE_COMPARE(Cr::Containers::triple(stats.nodeCount, stats.drawCount, stats.drawBatchCount), Cr::Containers::triple(data.expectedNodeCount, data.expectedDrawCount, data.expectedBatchCount));
}

}

CORRADE_TEST_MAIN(FloorplannerProcessingTest)
