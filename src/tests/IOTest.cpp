// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/Directory.h>
#include <gtest/gtest.h>
#include "esp/assets/RenderAssetInstanceCreationInfo.h"
#include "esp/core/esp.h"
#include "esp/io/JsonAllTypes.h"
#include "esp/io/io.h"
#include "esp/io/json.h"
#include "esp/metadata/attributes/ObjectAttributes.h"

#include "configure.h"

using namespace esp::io;

using esp::metadata::attributes::AbstractObjectAttributes;
using esp::metadata::attributes::ObjectAttributes;

const std::string dataDir =
    Corrade::Utility::Directory::join(SCENE_DATASETS, "../");

TEST(IOTest, fileExistTest) {
  std::string file = FILE_THAT_EXISTS;
  bool result = exists(file);
  EXPECT_TRUE(result);

  file = "Foo.bar";
  result = exists(file);
  EXPECT_FALSE(result);
}

TEST(IOTest, fileSizeTest) {
  std::string existingFile = FILE_THAT_EXISTS;
  auto result = fileSize(existingFile);
  LOG(INFO) << "File size of " << existingFile << " is " << result;

  std::string nonexistingFile = "Foo.bar";
  result = fileSize(nonexistingFile);
  LOG(INFO) << "File size of " << nonexistingFile << " is " << result;
}

TEST(IOTest, fileRmExtTest) {
  std::string filename = "/foo/bar.jpeg";

  // rm extension
  std::string result = removeExtension(filename);
  EXPECT_EQ(result, "/foo/bar");
  EXPECT_EQ(filename, "/foo/bar.jpeg");

  std::string filenameNoExt = "/path/to/foobar";
  result = removeExtension(filenameNoExt);
  EXPECT_EQ(result, filenameNoExt);
}

TEST(IOTest, fileReplaceExtTest) {
  std::string filename = "/foo/bar.jpeg";

  // change extension
  std::string ext = ".png";
  std::string result = changeExtension(filename, ext);

  EXPECT_EQ(result, "/foo/bar.png");

  std::string filenameNoExt = "/path/to/foobar";
  result = changeExtension(filenameNoExt, ext);
  EXPECT_EQ(result, "/path/to/foobar.png");

  std::string cornerCase = "";
  result = changeExtension(cornerCase, ext);
  EXPECT_EQ(result, ".png");

  cornerCase = ".";
  result = changeExtension(cornerCase, ext);
  EXPECT_EQ(result, "..png");

  cornerCase = "..";
  result = changeExtension(cornerCase, ext);
  EXPECT_EQ(result, "...png");

  std::string cornerCaseExt = "png";  // no dot
  result = changeExtension(filename, cornerCaseExt);
  EXPECT_EQ(result, "/foo/bar.png");

  cornerCase = ".";
  result = changeExtension(cornerCase, cornerCaseExt);
  EXPECT_EQ(result, "..png");

  cornerCase = "..";
  result = changeExtension(cornerCase, cornerCaseExt);
  EXPECT_EQ(result, "...png");

  cornerCase = ".jpg";
  result = changeExtension(cornerCase, cornerCaseExt);
  EXPECT_EQ(result, ".jpg.png");
}

TEST(IOTest, tokenizeTest) {
  std::string file = ",a,|,bb|c";
  const auto& t1 = tokenize(file, ",");
  EXPECT_EQ((std::vector<std::string>{"", "a", "|", "bb|c"}), t1);
  const auto& t2 = tokenize(file, "|");
  EXPECT_EQ((std::vector<std::string>{",a,", ",bb", "c"}), t2);
  const auto& t3 = tokenize(file, ",|", 0, true);
  EXPECT_EQ((std::vector<std::string>{"", "a", "bb", "c"}), t3);
}

/**
 * @brief Test basic JSON file processing
 */
TEST(IOTest, JsonTest) {
  std::string s = "{\"test\":[1,2,3,4]}";
  const auto& json = esp::io::parseJsonString(s);
  std::vector<int> t;
  esp::io::toIntVector(json["test"], &t);
  EXPECT_EQ(t[1], 2);
  EXPECT_EQ(esp::io::jsonToString(json), s);

  // test io
  auto testFilepath =
      Corrade::Utility::Directory::join(dataDir, "../io_test_json.json");
  EXPECT_TRUE(writeJsonToFile(json, testFilepath));
  const auto& loadedJson = esp::io::parseJsonFile(testFilepath);
  EXPECT_EQ(esp::io::jsonToString(loadedJson), s);
  Corrade::Utility::Directory::rm(testFilepath);

  // test basic attributes populating

  std::string attr_str =
      "{\"render mesh\": \"banana.glb\",\"join collision "
      "meshes\":false,\"mass\": 0.066,\"scale\": [2.0,2.0,2]}";

  // io::JsonGenericValue :
  esp::io::JsonDocument tmpJSON = esp::io::parseJsonString(attr_str);
  // io::JsonGenericValue :
  const esp::io::JsonGenericValue jsonDoc = tmpJSON.GetObject();

  // for function ptr placeholder
  using std::placeholders::_1;
  ObjectAttributes::ptr attributes = ObjectAttributes::create("temp");

  bool success = false;
  // test vector
  success = esp::io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonDoc, "scale", std::bind(&ObjectAttributes::setScale, attributes, _1));
  EXPECT_EQ(success, true);
  EXPECT_EQ(attributes->getScale()[1], 2);

  // test double
  success = esp::io::jsonIntoSetter<double>(
      jsonDoc, "mass", std::bind(&ObjectAttributes::setMass, attributes, _1));
  EXPECT_EQ(success, true);
  EXPECT_EQ(attributes->getMass(), 0.066);

  // test bool
  success = esp::io::jsonIntoSetter<bool>(
      jsonDoc, "join collision meshes",
      std::bind(&ObjectAttributes::setJoinCollisionMeshes, attributes, _1));
  EXPECT_EQ(success, true);
  EXPECT_EQ(attributes->getJoinCollisionMeshes(), false);

  // test string
  success = esp::io::jsonIntoSetter<std::string>(
      jsonDoc, "render mesh",
      std::bind(&ObjectAttributes::setRenderAssetHandle, attributes, _1));
  EXPECT_EQ(success, true);
  EXPECT_EQ(attributes->getRenderAssetHandle(), "banana.glb");
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

// Beware, ToRJsonValue/FromRJsonValue should generally go in JsonAllTypes.h,
// not scattered in user code as done here.
inline RJsonValue ToRJsonValue(const MyNestedStruct& x,
                               RJsonAllocator& allocator) {
  RJsonValue obj(rapidjson::kObjectType);
  AddMember(obj, "a", x.a, allocator);
  return obj;
}

void FromRJsonValue(const RJsonValue& obj, MyNestedStruct& x) {
  ReadMember(obj, "a", x.a);
}

inline RJsonValue ToRJsonValue(const MyOuterStruct& x,
                               RJsonAllocator& allocator) {
  RJsonValue obj(rapidjson::kObjectType);
  AddMember(obj, "nested", x.nested, allocator);
  AddMember(obj, "b", x.b, allocator);
  return obj;
}

void FromRJsonValue(const RJsonValue& obj, MyOuterStruct& x) {
  ReadMember(obj, "nested", x.nested);
  ReadMember(obj, "b", x.b);
}
}  // namespace

// Serialize/deserialize MyOuterStruct using io::AddMember/ReadMember and assert
// equality.
TEST(IOTest, JsonUserTypeTest) {
  rapidjson::Document d(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType& allocator = d.GetAllocator();

  MyOuterStruct myStruct{{"hello world"}, 2.f};
  AddMember(d, "myStruct", myStruct, allocator);

  MyOuterStruct myStruct2;
  ReadMember(d, "myStruct", myStruct2);

  ASSERT(myStruct2.nested.a == myStruct.nested.a);
  ASSERT(myStruct2.b == myStruct.b);
}

// Serialize/deserialize a few stl types using io::AddMember/ReadMember and
// assert equality.
TEST(IOTest, JsonStlTypesTest) {
  rapidjson::Document d(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType& allocator = d.GetAllocator();

  std::string s{"hello world"};
  AddMember(d, "s", s, allocator);
  std::string s2;
  ReadMember(d, "s", s2);
  ASSERT(s2 == s);

  // test a vector of ints
  std::vector<int> vec{3, 4, 5, 6};
  AddMember(d, "vec", vec, allocator);
  std::vector<int> vec2;
  ReadMember(d, "vec", vec2);
  ASSERT(vec2 == vec);

  // test an empty vector
  std::vector<float> emptyVec{};
  AddMember(d, "emptyVec", emptyVec, allocator);
  std::vector<float> emptyVec2;
  ReadMember(d, "emptyVec", emptyVec2);
  ASSERT(emptyVec2 == emptyVec);
}

// Serialize/deserialize a few esp types using io::AddMember/ReadMember and
// assert equality.
TEST(IOTest, JsonEspTypesTest) {
  rapidjson::Document d(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType& allocator = d.GetAllocator();

  // add RenderAssetInstanceCreationInfo
  esp::assets::RenderAssetInstanceCreationInfo creationInfo(
      "test_filepath", Magnum::Vector3(1.f, 2.f, 3.f),
      esp::assets::RenderAssetInstanceCreationInfo::Flags(),
      "test_light_setup");
  AddMember(d, "creationInfo", creationInfo, allocator);

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
  AddMember(d, "assetInfo", assetInfo, allocator);

  // add RenderAssetInstanceState
  esp::gfx::replay::RenderAssetInstanceState state{
      {Magnum::Vector3(1.f, 2.f, 3.f),
       Magnum::Quaternion::rotation(Magnum::Rad{1.f},
                                    Magnum::Vector3(0.f, 1.f, 0.f))},
      4};
  AddMember(d, "state", state, allocator);

  // read and compare RenderAssetInstanceCreationInfo
  esp::assets::RenderAssetInstanceCreationInfo creationInfo2;
  ReadMember(d, "creationInfo", creationInfo2);
  ASSERT(creationInfo2.filepath == creationInfo.filepath);
  ASSERT(creationInfo2.scale == creationInfo.scale);
  ASSERT(creationInfo2.flags == creationInfo.flags);
  ASSERT(creationInfo2.lightSetupKey == creationInfo.lightSetupKey);

  // read and compare AssetInfo
  esp::assets::AssetInfo assetInfo2;
  ReadMember(d, "assetInfo", assetInfo2);
  ASSERT(assetInfo2.type == assetInfo.type);
  ASSERT(assetInfo2.filepath == assetInfo.filepath);
  ASSERT(assetInfo2.frame.up() == assetInfo.frame.up());
  ASSERT(assetInfo2.frame.front() == assetInfo.frame.front());
  ASSERT(assetInfo2.frame.origin() == assetInfo.frame.origin());
  ASSERT(assetInfo2.virtualUnitToMeters == assetInfo.virtualUnitToMeters);
  ASSERT(assetInfo2.requiresLighting == assetInfo.requiresLighting);
  ASSERT(assetInfo2.splitInstanceMesh == assetInfo.splitInstanceMesh);

  // read and compare RenderAssetInstanceState
  esp::gfx::replay::RenderAssetInstanceState state2;
  ReadMember(d, "state", state2);
  ASSERT(state2 == state);
}
