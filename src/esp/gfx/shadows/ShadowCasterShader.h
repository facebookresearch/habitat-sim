#pragma once
/*

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019 —
            Vladimír Vondruš <mosra@centrum.cz>
        2016 — Bill Robinson <airbaggins@gmail.com>
*/

#include <Magnum/GL/AbstractShaderProgram.h>

namespace esp {
namespace gfx {

class ShadowCasterShader : public Magnum::GL::AbstractShaderProgram {
 public:
  explicit ShadowCasterShader();

  /**
   * @brief Set transformation matrix
   *
   * Matrix that transforms from local model space -> world space ->
   * camera space -> clip coordinates (aka model-view-projection
   * matrix).
   */
  ShadowCasterShader& setTransformationMatrix(const Magnum::Matrix4& matrix);

 private:
  Magnum::Int _transformationMatrixUniform;
};

}  // namespace gfx
}  // namespace esp
