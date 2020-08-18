// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Directory.h>
#include <gtest/gtest.h>
#include "esp/core/esp.h"
#include "esp/io/URDFParser.h"
#include "esp/io/io.h"

#include "configure.h"

namespace Cr = Corrade;

using namespace esp::io;

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

TEST(IOTest, parseURDF) {
  const std::string iiwaURDF = Cr::Utility::Directory::join(
      TEST_ASSETS, "URDF/kuka_iiwa/model_free_base.urdf");

  URDF::Parser parser;

  // load the iiwa test asset
  parser.parseURDF(iiwaURDF);
  auto& model = parser.getModel();
  Cr::Utility::Debug() << "name: " << model.m_name;
  EXPECT_EQ(model.m_name, "lbr_iiwa");
  Cr::Utility::Debug() << "file: " << model.m_sourceFile;
  EXPECT_EQ(model.m_sourceFile, iiwaURDF);
  Cr::Utility::Debug() << "links: " << model.m_links;
  EXPECT_EQ(model.m_links.size(), 8);
  Cr::Utility::Debug() << "root links: " << model.m_rootLinks;
  EXPECT_EQ(model.m_rootLinks.size(), 1);
  Cr::Utility::Debug() << "joints: " << model.m_joints;
  EXPECT_EQ(model.m_joints.size(), 7);
  Cr::Utility::Debug() << "materials: " << model.m_materials;
  EXPECT_EQ(model.m_materials.size(), 3);

  // test overwrite re-load
  parser.parseURDF(iiwaURDF);
}
