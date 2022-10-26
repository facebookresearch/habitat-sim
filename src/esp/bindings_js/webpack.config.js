// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

const path = require("path");
const HtmlWebpackPlugin = require("html-webpack-plugin");

// Parse the args
const args = require("minimist")(process.argv.slice(2));

if (!args.build_dir) {
  console.log("Specifying --build_dir option is required");
  process.exit(1);
}

// All of the files will be outputted in build_dir argument passed
// from command line
const buildRootPath = path.resolve(__dirname, args.build_dir);
console.log("Build Directory is", buildRootPath);

const config = {
  target: "web",
  entry: path.resolve(__dirname, "index.js"),
  output: {
    path: buildRootPath,
    filename: "bundle.js"
  },
  module: {
    rules: [
      // Babel loader for > ES6 compilation
      // Will also run eslint
      {
        test: /\.(js|ts)$/,
        exclude: /node_modules/,
        use: ["babel-loader", "eslint-loader"]
      },
      // CSS Files
      {
        test: /\.css$/,
        use: ["style-loader", "css-loader"]
      }
    ]
  },
  // No need to add .js at the end of the module names with this
  resolve: {
    extensions: [".js", ".ts"]
  },
  plugins: [
    new HtmlWebpackPlugin({
      template: path.resolve(__dirname, "bindings.html"),
      filename: path.resolve(buildRootPath, "bindings.html"),
      hash: true
    }),
    new HtmlWebpackPlugin({
      template: path.resolve(__dirname, "test_page.html"),
      filename: path.resolve(buildRootPath, "test_page.html"),
      hash: true
    }),
    new HtmlWebpackPlugin({
      template: path.resolve(__dirname, "viewer.html"),
      filename: path.resolve(buildRootPath, "viewer.html"),
      hash: true
    })
  ],
  devtool: "source-map"
};

module.exports = config;
