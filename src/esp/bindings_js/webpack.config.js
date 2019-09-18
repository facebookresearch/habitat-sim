const path = require('path');
const HtmlWebpackPlugin = require('html-webpack-plugin')

// All of the files will be outputted in build_js
const buildRootPath = path.resolve(__dirname).replace('src', 'build_js');

const config = {
    target: 'web',
    entry: path.resolve(__dirname, 'index.js'),
    output: {
        path: buildRootPath,
        filename: 'bundle.js'
    },
    // Fix around missing fs dependency error
    node: { fs: 'empty' },
    module: {
        rules: [
            // Babel loader for > ES6 compilation
            // Will also run eslint
            {
                test: /\.(js)$/,
                exclude: /node_modules/,
                use: ['babel-loader', 'eslint-loader']
            },
            // CSS Files
            {
                test: /\.css$/,
                use: ['style-loader', 'css-loader'],
            }
        ]
    },
    // No need to add .js at the end of the module names with this
    resolve: {
        extensions: ['.js']
    },
    plugins: [
        new HtmlWebpackPlugin({
            template: path.resolve(__dirname, 'bindings.html'),
            filename: path.resolve(buildRootPath, 'bindings.html'),
            hash: true
        })
    ],
    devtool: 'source-map'
}

module.exports = config;