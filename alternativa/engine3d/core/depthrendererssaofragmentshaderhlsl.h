#ifndef DEPTHRENDERERSSAOFRAGMENTSHADERHLSL_H
#define DEPTHRENDERERSSAOFRAGMENTSHADERHLSL_H

class QByteArray;
class ShaderSampler;

QByteArray DepthRendererSSAOFragmentShaderHLSL(ShaderSampler &samplers, int count);

#endif // DEPTHRENDERERSSAOFRAGMENTSHADERHLSL_H
