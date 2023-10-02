// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Path.h>

#include "esp/assets/RenderAssetInstanceCreationInfo.h"
#include "esp/core/Esp.h"
#include "esp/io/Io.h"
#include "esp/io/Json.h"
#include "esp/io/JsonAllTypes.h"
#include "esp/metadata/URDFParser.h"
#include "esp/metadata/attributes/ArticulatedObjectAttributes.h"
#include "esp/metadata/attributes/ObjectAttributes.h"

#include "configure.h"

namespace Cr = Corrade;

using esp::metadata::attributes::ArticulatedObjectAttributes;
using esp::metadata::attributes::ObjectAttributes;
namespace {
const std::string dataDir = Corrade::Utility::Path::join(SCENE_DATASETS, "../");

struct IOTest : Cr::TestSuite::Tester {
  explicit IOTest();
  void fileReplaceExtTest();
  void testEllipsisFilter();
  void parseURDF();
  void testJson();
  void testJsonBuiltinTypes();
  void testJsonStlTypes();
  void testJsonMagnumTypes();
  void testJsonEspTypes();

  void testJsonUserType();

  template <typename T>
  void _testJsonReadWrite(T src,
                          rapidjson::GenericStringRef<char> name,
                          rapidjson::Document& d) {
    esp::io::addMember(d, name, src, d.GetAllocator());
    T src2;
    CORRADE_VERIFY(esp::io::readMember(d, name, src2));
    CORRADE_COMPARE(src2, src);
  }

  esp::logging::LoggingContext loggingContext;
};

IOTest::IOTest() {
  addTests({&IOTest::fileReplaceExtTest, &IOTest::testEllipsisFilter,
            &IOTest::parseURDF, &IOTest::testJson,
            &IOTest::testJsonBuiltinTypes, &IOTest::testJsonStlTypes,
            &IOTest::testJsonMagnumTypes, &IOTest::testJsonEspTypes,
            &IOTest::testJsonUserType});
}

void IOTest::fileReplaceExtTest() {
  std::string filename = "/foo/bar.jpeg";

  // change extension
  std::string ext = ".png";
  std::string result = esp::io::changeExtension(filename, ext);

  CORRADE_COMPARE(result, "/foo/bar.png");

  std::string filenameNoExt = "/path/to/foobar";
  result = esp::io::changeExtension(filenameNoExt, ext);
  CORRADE_COMPARE(result, "/path/to/foobar.png");

  std::string cornerCase = "";
  result = esp::io::changeExtension(cornerCase, ext);
  CORRADE_COMPARE(result, ".png");

  cornerCase = ".";
  result = esp::io::changeExtension(cornerCase, ext);
  CORRADE_COMPARE(result, "..png");

  cornerCase = "..";
  result = esp::io::changeExtension(cornerCase, ext);
  CORRADE_COMPARE(result, "...png");

  std::string cornerCaseExt = "png";  // no dot
  result = esp::io::changeExtension(filename, cornerCaseExt);
  CORRADE_COMPARE(result, "/foo/bar.png");

  cornerCase = ".";
  result = esp::io::changeExtension(cornerCase, cornerCaseExt);
  CORRADE_COMPARE(result, "..png");

  cornerCase = "..";
  result = esp::io::changeExtension(cornerCase, cornerCaseExt);
  CORRADE_COMPARE(result, "...png");

  cornerCase = ".jpg";
  result = esp::io::changeExtension(cornerCase, cornerCaseExt);
  CORRADE_COMPARE(result, ".jpg.png");
}

void IOTest::testEllipsisFilter() {
  std::string testPath1 = "/test/path/to/test/no-op.txt";
  std::string res = esp::io::normalizePath(testPath1);
  CORRADE_COMPARE(res, "/test/path/to/test/no-op.txt");

  testPath1 = "/test/path/DELETE/../to/test/removal.txt";
  res = esp::io::normalizePath(testPath1);
  CORRADE_COMPARE(res, "/test/path/to/test/removal.txt");

  testPath1 =
      "/test/path/DELETE/../to/DELETE/../test/DELETE/../multiple/removal.txt";
  res = esp::io::normalizePath(testPath1);
  CORRADE_COMPARE(res, "/test/path/to/test/multiple/removal.txt");

  testPath1 = "test/path/DELETE/../to/test/removal.txt";
  res = esp::io::normalizePath(testPath1);
  CORRADE_COMPARE(res, "test/path/to/test/removal.txt");

  testPath1 =
      "test/path/DELETE/../to/DELETE/../test/DELETE/../multiple/removal.txt";
  res = esp::io::normalizePath(testPath1);
  CORRADE_COMPARE(res, "test/path/to/test/multiple/removal.txt");

  testPath1 =
      "/test/path/DELETE/DELETE2/DELETE3/../../../to/DELETE/DELETE2/../../test/"
      "DELETE/../consecutive/removal.txt";
  res = esp::io::normalizePath(testPath1);
  CORRADE_COMPARE(res, "/test/path/to/test/consecutive/removal.txt");

  // ignore intitial ellipsis
  testPath1 = "/../test/path/DELETE/../to/test/initial/ignore.txt";
  res = esp::io::normalizePath(testPath1);
  CORRADE_COMPARE(res, "/../test/path/DELETE/../to/test/initial/ignore.txt");

}  // IOTest::testEllipsisFilter

void IOTest::parseURDF() {
  const std::string iiwaURDF = Cr::Utility::Path::join(
      TEST_ASSETS, "urdf/kuka_iiwa/model_free_base.urdf");

  ArticulatedObjectAttributes::ptr attributes =
      ArticulatedObjectAttributes::create(iiwaURDF);

  esp::metadata::URDF::Parser parser;

  // load the iiwa test asset
  std::shared_ptr<esp::metadata::URDF::Model> urdfModel;
  parser.parseURDF(attributes, urdfModel);
  ESP_DEBUG() << "name:" << urdfModel->m_name;
  CORRADE_COMPARE(urdfModel->m_name, "lbr_iiwa");
  ESP_DEBUG() << "file:" << urdfModel->m_sourceFile;
  CORRADE_COMPARE(urdfModel->m_sourceFile, iiwaURDF);
  ESP_DEBUG() << "links:" << urdfModel->m_links;
  CORRADE_COMPARE(urdfModel->m_links.size(), 8);
  ESP_DEBUG() << "root links:" << urdfModel->m_rootLinks;
  CORRADE_COMPARE(urdfModel->m_rootLinks.size(), 1);
  ESP_DEBUG() << "joints:" << urdfModel->m_joints;
  CORRADE_COMPARE(urdfModel->m_joints.size(), 7);
  ESP_DEBUG() << "materials:" << urdfModel->m_materials;
  CORRADE_COMPARE(urdfModel->m_materials.size(), 3);

  // check global scaling
  CORRADE_COMPARE(urdfModel->getGlobalScaling(), 1.0);
  // this link is a mesh shape, so check the mesh scale
  CORRADE_COMPARE(
      urdfModel->getLink(1)->m_collisionArray.back().m_geometry.m_type, 5);
  CORRADE_COMPARE(
      urdfModel->getLink(1)->m_collisionArray.back().m_geometry.m_meshScale,
      Mn::Vector3{1.0});
  urdfModel->setGlobalScaling(2.0);
  CORRADE_COMPARE(urdfModel->getGlobalScaling(), 2.0);
  CORRADE_COMPARE(
      urdfModel->getLink(1)->m_collisionArray.back().m_geometry.m_meshScale,
      Mn::Vector3{2.0});

  // check mass scaling
  CORRADE_COMPARE(urdfModel->getMassScaling(), 1.0);
  CORRADE_COMPARE(urdfModel->getLink(1)->m_inertia.m_mass, 4.0);
  urdfModel->setMassScaling(3.0);
  CORRADE_COMPARE(urdfModel->getMassScaling(), 3.0);
  CORRADE_COMPARE(urdfModel->getLink(1)->m_inertia.m_mass, 12.0);

  // test overwrite re-load
  parser.parseURDF(attributes, urdfModel);
  // should have default values again
  CORRADE_COMPARE(urdfModel->getGlobalScaling(), 1.0);
  CORRADE_COMPARE(urdfModel->getMassScaling(), 1.0);
  CORRADE_COMPARE(
      urdfModel->getLink(1)->m_collisionArray.back().m_geometry.m_meshScale,
      Mn::Vector3{1.0});
  CORRADE_COMPARE(urdfModel->getLink(1)->m_inertia.m_mass, 4.0);
}

/**
 * @brief Test basic JSON file processing
 */
void IOTest::testJson() {
  std::string s = "{\"test\":[1,2,3,4]}";
  const auto& json = esp::io::parseJsonString(s);
  std::vector<int> t;
  esp::io::toIntVector(json["test"], &t);
  CORRADE_COMPARE(t[1], 2);
  CORRADE_COMPARE(esp::io::jsonToString(json), s);

  // test io
  auto testFilepath =
      Corrade::Utility::Path::join(dataDir, "../io_test_json.json");
  CORRADE_VERIFY(esp::io::writeJsonToFile(json, testFilepath));
  const auto& loadedJson = esp::io::parseJsonFile(testFilepath);
  CORRADE_COMPARE(esp::io::jsonToString(loadedJson), s);
  Corrade::Utility::Path::remove(testFilepath);

  // test basic attributes populating

  std::string attr_str =
      "{\"render asset\": \"banana.glb\",\"join collision "
      "meshes\":false,\"mass\": 0.066,\"scale\": [2.0,2.0,2]}";

  // io::JsonGenericValue :
  esp::io::JsonDocument tmpJSON = esp::io::parseJsonString(attr_str);
  // io::JsonGenericValue :
  const esp::io::JsonGenericValue jsonDoc = tmpJSON.GetObject();

  ObjectAttributes::ptr attributes = ObjectAttributes::create("temp");

  bool success = false;
  // test vector
  success = esp::io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonDoc, "scale", [attributes](const Magnum::Vector3& scale) {
        attributes->setScale(scale);
      });
  CORRADE_COMPARE(success, true);
  CORRADE_COMPARE(attributes->getScale()[1], 2);

  // test double
  success = esp::io::jsonIntoSetter<double>(
      jsonDoc, "mass",
      [attributes](double mass) { attributes->setMass(mass); });
  CORRADE_COMPARE(success, true);
  CORRADE_COMPARE(attributes->getMass(), 0.066);

  // test bool
  success = esp::io::jsonIntoSetter<bool>(
      jsonDoc, "join collision meshes",
      [attributes](bool join_collision_meshes) {
        attributes->setJoinCollisionMeshes(join_collision_meshes);
      });
  CORRADE_COMPARE(success, true);
  CORRADE_COMPARE(attributes->getJoinCollisionMeshes(), false);

  // test string
  success = esp::io::jsonIntoConstSetter<std::string>(
      jsonDoc, "render asset", [attributes](const std::string& render_asset) {
        attributes->setRenderAssetHandle(render_asset);
      });
  CORRADE_COMPARE(success, true);
  CORRADE_COMPARE(attributes->getRenderAssetHandle(), "banana.glb");
}

// Serialize/deserialize the 7 rapidjson builtin types using
// esp::io::addMember/esp::io::readMember and assert equality.
void IOTest::testJsonBuiltinTypes() {
  rapidjson::Document d(rapidjson::kObjectType);
  // specify this as the test
  CORRADE_VERIFY(true);

  _testJsonReadWrite(std::numeric_limits<int>::lowest(), "myint", d);

  _testJsonReadWrite(std::numeric_limits<unsigned>::max(), "myunsigned", d);

  _testJsonReadWrite(std::numeric_limits<int64_t>::lowest(), "myint64_t", d);

  _testJsonReadWrite(std::numeric_limits<uint64_t>::max(), "myuint64_t", d);

  _testJsonReadWrite(1.0f / 7.0f, "myfloat", d);

  _testJsonReadWrite(double{1.0 / 13}, "mydouble", d);

  _testJsonReadWrite(bool{true}, "mybool", d);

  // verify failure to read bool into int
  int x_err{0};
  CORRADE_VERIFY(!esp::io::readMember(d, "mybool", x_err));

  // verify failure to read missing tag
  int x_err2{0};
  CORRADE_VERIFY(!esp::io::readMember(d, "my_missing_int", x_err2));
}

// Serialize/deserialize a few stl types using
// esp::io::addMember/esp::io::readMember and assert equality.
void IOTest::testJsonStlTypes() {
  rapidjson::Document d(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType& allocator = d.GetAllocator();

  std::string s{"hello world"};
  _testJsonReadWrite(s, "s", d);

  std::pair<float, std::string> pair(1.5f, "second");
  esp::io::addMember(d, "pair", pair, allocator);
  std::pair<float, std::string> pair2;
  CORRADE_VERIFY(esp::io::readMember(d, "pair", pair2));
  CORRADE_COMPARE(pair2, pair);

  // test a vector of ints
  _testJsonReadWrite(std::vector<int>{3, 4, 5, 6}, "vec", d);

  // test an empty vector
  _testJsonReadWrite(std::vector<float>{}, "emptyVec", d);

  // test reading a vector of wrong type
  std::vector<std::string> vec3;
  CORRADE_VERIFY(!esp::io::readMember(d, "vec", vec3));
}

// Serialize/deserialize a few Magnum types using
// esp::io::addMember/esp::io::readMember and
// assert equality.
void IOTest::testJsonMagnumTypes() {
  rapidjson::Document d(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType& allocator = d.GetAllocator();
  // specify this as the test
  CORRADE_VERIFY(true);

  _testJsonReadWrite(Magnum::Vector3{1, 2, 3}, "myvec", d);

  _testJsonReadWrite(Magnum::Quaternion{{1, 2, 3}, 4}, "myquat", d);

  _testJsonReadWrite(Magnum::Matrix3{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}, "mymat",
                     d);

  // test reading the wrong type (wrong number of fields)
  Magnum::Quaternion quat3;
  CORRADE_VERIFY(!esp::io::readMember(d, "myvec", quat3));

  // test reading the wrong type (wrong number of fields)
  Magnum::Vector3 vec3;
  CORRADE_VERIFY(!esp::io::readMember(d, "myquat", vec3));

  // test reading the wrong type (array elements aren't numbers)
  std::vector<std::string> vecOfStrings{"1", "2", "3"};
  esp::io::addMember(d, "myVecOfStrings", vecOfStrings, allocator);
  CORRADE_VERIFY(!esp::io::readMember(d, "myVecOfStrings", vec3));
}

// Serialize/deserialize a few esp types using
// esp::io::addMember/esp::io::readMember and assert equality.
void IOTest::testJsonEspTypes() {
  rapidjson::Document d(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType& allocator = d.GetAllocator();
  // specify this as the test
  CORRADE_VERIFY(true);

  {
    // vec3f
    _testJsonReadWrite(esp::vec3f{1, 2, 3}, "myvec3f", d);

    // test reading the wrong type (wrong number of fields)
    std::vector<float> wrongNumFieldsVec{1, 3, 4, 4};
    esp::io::addMember(d, "mywrongNumFieldsVec", wrongNumFieldsVec, allocator);
    esp::vec3f vec3;
    CORRADE_VERIFY(!esp::io::readMember(d, "mywrongNumFieldsVec", vec3));

    // test reading the wrong type (array elements aren't numbers)
    std::vector<std::string> vecOfStrings{"1", "2", "3"};
    esp::io::addMember(d, "myVecOfStrings", vecOfStrings, allocator);
    CORRADE_VERIFY(!esp::io::readMember(d, "myVecOfStrings", vec3));
  }

  {
    // RenderAssetInstanceCreationInfo
    esp::assets::RenderAssetInstanceCreationInfo creationInfo(
        "test_filepath", Magnum::Vector3(1.f, 2.f, 3.f),
        esp::assets::RenderAssetInstanceCreationInfo::Flags(),
        "test_light_setup");
    esp::io::addMember(d, "creationInfo", creationInfo, allocator);
    esp::assets::RenderAssetInstanceCreationInfo creationInfo2;
    CORRADE_VERIFY(esp::io::readMember(d, "creationInfo", creationInfo2));
    CORRADE_COMPARE(creationInfo2.filepath, creationInfo.filepath);
    CORRADE_VERIFY(creationInfo2.scale == creationInfo.scale);
    CORRADE_VERIFY(creationInfo2.flags == creationInfo.flags);
    CORRADE_VERIFY(creationInfo2.lightSetupKey == creationInfo.lightSetupKey);
  }

  {
    // AssetInfo
    esp::assets::AssetInfo assetInfo{
        esp::assets::AssetType::MP3D_MESH,
        "test_filepath2",
        esp::geo::CoordinateFrame(esp::vec3f(1.f, 0.f, 0.f),
                                  esp::vec3f(0.f, 0.f, 1.f),
                                  esp::vec3f(1.f, 2.f, 3.f)),
        4.f,
        true,
        false};
    esp::io::addMember(d, "assetInfo", assetInfo, allocator);
    esp::assets::AssetInfo assetInfo2;
    CORRADE_VERIFY(esp::io::readMember(d, "assetInfo", assetInfo2));

    CORRADE_VERIFY(assetInfo2.type == assetInfo.type);
    CORRADE_COMPARE(assetInfo2.filepath, assetInfo.filepath);
    CORRADE_VERIFY(assetInfo2.frame.up().isApprox(assetInfo.frame.up()));
    CORRADE_VERIFY(assetInfo2.frame.front().isApprox(assetInfo.frame.front()));
    CORRADE_VERIFY(
        assetInfo2.frame.origin().isApprox(assetInfo.frame.origin()));
    CORRADE_COMPARE(assetInfo2.virtualUnitToMeters,
                    assetInfo.virtualUnitToMeters);
    CORRADE_COMPARE(assetInfo2.forceFlatShading, assetInfo.forceFlatShading);
    CORRADE_COMPARE(assetInfo2.splitInstanceMesh, assetInfo.splitInstanceMesh);
    CORRADE_VERIFY(assetInfo2.overridePhongMaterial == Cr::Containers::NullOpt);
    // now test again with override material
    assetInfo.overridePhongMaterial = esp::assets::PhongMaterialColor();
    assetInfo.overridePhongMaterial->ambientColor =
        Mn::Color4(0.1, 0.2, 0.3, 0.4);
    assetInfo.overridePhongMaterial->diffuseColor =
        Mn::Color4(0.2, 0.3, 0.4, 0.5);
    assetInfo.overridePhongMaterial->specularColor =
        Mn::Color4(0.3, 0.4, 0.5, 0.6);
    CORRADE_VERIFY(assetInfo.overridePhongMaterial);
    esp::io::addMember(d, "assetInfoColorOverride", assetInfo, allocator);
    CORRADE_VERIFY(
        esp::io::readMember(d, "assetInfoColorOverride", assetInfo2));
    CORRADE_VERIFY(assetInfo2.overridePhongMaterial);
    CORRADE_COMPARE(assetInfo2.overridePhongMaterial->ambientColor,
                    assetInfo.overridePhongMaterial->ambientColor);
    CORRADE_COMPARE(assetInfo2.overridePhongMaterial->diffuseColor,
                    assetInfo.overridePhongMaterial->diffuseColor);
    CORRADE_COMPARE(assetInfo2.overridePhongMaterial->specularColor,
                    assetInfo.overridePhongMaterial->specularColor);
  }

  {
    // RenderAssetInstanceState
    esp::gfx::replay::RenderAssetInstanceState state{
        {Magnum::Vector3(1.f, 2.f, 3.f),
         Magnum::Quaternion::rotation(Magnum::Rad{1.f},
                                      Magnum::Vector3(0.f, 1.f, 0.f))},
        4};
    esp::io::addMember(d, "state", state, allocator);
    // read and compare RenderAssetInstanceState
    esp::gfx::replay::RenderAssetInstanceState state2;
    CORRADE_VERIFY(esp::io::readMember(d, "state", state2));
    CORRADE_VERIFY(state2 == state);
  }
}

namespace {
// some test structs for JsonUserTypeTest below
struct MyNestedStruct {
  std::string a;
};

struct MyOuterStruct {
  MyNestedStruct nested;
  float b;
};

// Beware, toJsonValue/fromJsonValue should generally go in JsonAllTypes.h,
// not scattered in user code as done here.
inline esp::io::JsonGenericValue toJsonValue(
    const MyNestedStruct& x,
    esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "a", x.a, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, MyNestedStruct& x) {
  esp::io::readMember(obj, "a", x.a);
  return true;
}

inline esp::io::JsonGenericValue toJsonValue(
    const MyOuterStruct& x,
    esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "nested", x.nested, allocator);
  esp::io::addMember(obj, "b", x.b, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, MyOuterStruct& x) {
  esp::io::readMember(obj, "nested", x.nested);
  esp::io::readMember(obj, "b", x.b);
  return true;
}
}  // namespace

// Serialize/deserialize MyOuterStruct using
// esp::io::addMember/esp::io::readMember and assert
// equality.
void IOTest::testJsonUserType() {
  rapidjson::Document d(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType& allocator = d.GetAllocator();

  MyOuterStruct myStruct{{"hello world"}, 2.f};
  esp::io::addMember(d, "myStruct", myStruct, allocator);

  MyOuterStruct myStruct2;
  esp::io::readMember(d, "myStruct", myStruct2);

  CORRADE_COMPARE(myStruct2.nested.a, myStruct.nested.a);
  CORRADE_COMPARE(myStruct2.b, myStruct.b);
}

}  // namespace

CORRADE_TEST_MAIN(IOTest)
