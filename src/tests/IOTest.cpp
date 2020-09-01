// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <gtest/gtest.h>
#include "esp/assets/attributes/ObjectAttributes.h"
#include "esp/core/esp.h"
#include "esp/io/io.h"
#include "esp/io/json.h"

#include "configure.h"

using namespace esp::io;

using esp::assets::attributes::AbstractObjectAttributes;
using esp::assets::attributes::ObjectAttributes;

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
  std::string s = "{\"test\":[1, 2, 3, 4]}";
  const auto& json = esp::io::parseJsonString(s);
  std::vector<int> t;
  esp::io::toIntVector(json["test"], &t);
  EXPECT_EQ(t[1], 2);
  EXPECT_EQ(esp::io::jsonToString(json), "{\"test\":[1,2,3,4]}");

  // test basic attributes populating

  std::string attr_str =
      "{\"render mesh\": \"banana.glb\",\"join collision "
      "meshes\":false,\"mass\": 0.066,\"scale\": [2.0,2.0,2]}";

  const auto& jsonDoc = esp::io::parseJsonString(attr_str);

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
