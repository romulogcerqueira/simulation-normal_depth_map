#include "FXAA.hpp"

#include <osg/Node>
#include <osg/Program>
#include <osg/ref_ptr>
#include <osg/Shader>
#include <osg/StateSet>
#include <osg/Uniform>
#include <osg/Texture>
#include <osg/Texture2D>
#include <osgDB/FileUtils>

namespace vizkit3d_normal_depth_map {

#define PATH_SHADER_FXAA_FRAG "vizkit3d_normal_depth_map/shaders/fxaa.frag"
#define PATH_SHADER_FXAA_VERT "vizkit3d_normal_depth_map/shaders/fxaa.vert"

FXAA::FXAA(){
  _fxaaShaderNode = createFXAAShaderNode();
}

FXAA::FXAA(float fxaa_span_max, float fxaa_reduce_mul, float fxaa_reduce_min){
  _fxaaShaderNode = createFXAAShaderNode( fxaa_span_max, fxaa_reduce_mul,
                                          fxaa_reduce_min);
};

float FXAA::getFxaaSpanMax(){
  return 0;
}

void FXAA::setFxaaSpanMax( float fxaa_span_max){
  _fxaaShaderNode->getOrCreateStateSet()->
    getUniform("fxaa_span_max")->set(fxaa_span_max);
}

float FXAA::getFxaaReduceMul(){
  return 0;
}
void FXAA::setFxaaReduceMul( float fxaa_reduce_mul){
  _fxaaShaderNode->getOrCreateStateSet()->
    getUniform("fxaa_reduce_mul")->set(fxaa_reduce_mul);
}

float FXAA::getFxaaReduceMin(){
  return 0;
}

void FXAA::setFxaaReduceMin( float fxaa_reduce_min){
  _fxaaShaderNode->getOrCreateStateSet()->
    getUniform("fxaa_reduce_min")->set(fxaa_reduce_min);
}

void FXAA::addImageToFxaa( osg::ref_ptr<osg::Image> image){
  insertTextureInStateSet(image, _fxaaShaderNode->getOrCreateStateSet());
};



osg::ref_ptr<osg::Group> FXAA::createFXAAShaderNode(  float fxaa_span_max,
                                                      float fxaa_reduce_mul,
                                                      float fxaa_reduce_min) {

  osg::ref_ptr<osg::Group> localRoot = new osg::Group();
  osg::ref_ptr<osg::Program> program(new osg::Program());

  // osg::ref_ptr<osg::Shader> shaderVertex =
  //   osg::Shader::readShaderFile(osg::Shader::VERTEX,
  //                               osgDB::findDataFile(SHADER_PATH_VERT));

  osg::ref_ptr<osg::Shader> shaderFragment =
    osg::Shader::readShaderFile(osg::Shader::FRAGMENT,
                                osgDB::findDataFile(PATH_SHADER_FXAA_FRAG));

  // program->addShader(shaderVertex);
  program->addShader(shaderFragment);

  osg::ref_ptr<osg::StateSet> ss = localRoot->getOrCreateStateSet();
  ss->setAttribute(program);

  osg::ref_ptr<osg::Uniform> fxaa_span_max_uniform(
                          new osg::Uniform("fxaa_span_max", fxaa_span_max));
  ss->addUniform(fxaa_span_max_uniform);

  osg::ref_ptr<osg::Uniform> fxaa_reduce_mul_uniform(
                          new osg::Uniform("fxaa_reduce_mul", fxaa_reduce_mul));
  ss->addUniform(fxaa_reduce_mul_uniform);

  osg::ref_ptr<osg::Uniform> fxaa_reduce_min_uniform(
                          new osg::Uniform("fxaa_reduce_min", fxaa_reduce_min));
  ss->addUniform(fxaa_reduce_min_uniform);

  return localRoot;
}




void FXAA::insertTextureInStateSet( osg::ref_ptr<osg::Image> image,
                                    osg::StateSet *state_set ){

  osg::ref_ptr<osg::Texture2D> image_tex = new osg::Texture2D();

  image_tex->setImage(image);
  image_tex->setDataVariance(osg::Object::DYNAMIC);
  image_tex->setFilter( osg::Texture::MIN_FILTER,
                        osg::Texture::LINEAR_MIPMAP_LINEAR);
  image_tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
  image_tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP);
  image_tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP);
  image_tex->setResizeNonPowerOfTwoHint(false);
  image_tex->setMaxAnisotropy(8.0f);

  int image_tex_unit = 0;
  state_set->setTextureAttributeAndModes(image_tex_unit, image_tex, osg::StateAttribute::ON);
  state_set->addUniform(new osg::Uniform("original_image", image_tex_unit));

}

} // namespace vizkit3d_normal_depth_map
