uniform sampler2D buf0;
uniform vec2 frameBufSize;
uniform vec2 texCoords;

uniform float fxaa_span_max;
uniform float fxaa_reduce_mul;
uniform float fxaa_reduce_min;

void main( void ) {
  //gl_FragColor.xyz = texture2D(buf0,texCoords).xyz;
  //return;
 
  //float fxaa_span_max = 8.0;
  //float fxaa_reduce_mul = 1.0/8.0;
  //float fxaa_reduce_min = 1.0/128.0;

  vec3 rgbNW=texture2D(buf0,texCoords+(vec2(-1.0,-1.0)/frameBufSize)).xyz;
  vec3 rgbNE=texture2D(buf0,texCoords+(vec2(1.0,-1.0)/frameBufSize)).xyz;
  vec3 rgbSW=texture2D(buf0,texCoords+(vec2(-1.0,1.0)/frameBufSize)).xyz;
  vec3 rgbSE=texture2D(buf0,texCoords+(vec2(1.0,1.0)/frameBufSize)).xyz;
  vec3 rgbM=texture2D(buf0,texCoords).xyz;

  vec3 luma=vec3(0.299, 0.587, 0.114);
  float lumaNW = dot(rgbNW, luma);
  float lumaNE = dot(rgbNE, luma);
  float lumaSW = dot(rgbSW, luma);
  float lumaSE = dot(rgbSE, luma);
  float lumaM  = dot(rgbM,  luma);

  float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
  float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));

  vec2 dir;
  dir.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));
  dir.y =  ((lumaNW + lumaSW) - (lumaNE + lumaSE));

  float dirReduce = max(
                (lumaNW + lumaNE + lumaSW + lumaSE) * (0.25 * fxaa_reduce_mul),
                fxaa_reduce_min);

  float rcpDirMin = 1.0/(min(abs(dir.x), abs(dir.y)) + dirReduce);

  dir = min( vec2( fxaa_span_max,  fxaa_span_max),
             max( vec2(-fxaa_span_max, -fxaa_span_max),
                  dir * rcpDirMin)) / frameBufSize;

  vec3 rgbA = (1.0/2.0) * (
                    texture2D(buf0, texCoords.xy + dir * (1.0/3.0 - 0.5)).xyz +
                    texture2D(buf0, texCoords.xy + dir * (2.0/3.0 - 0.5)).xyz);
  vec3 rgbB = rgbA * (1.0/2.0) + (1.0/4.0) * (
                    texture2D(buf0, texCoords.xy + dir * (0.0/3.0 - 0.5)).xyz +
                    texture2D(buf0, texCoords.xy + dir * (3.0/3.0 - 0.5)).xyz);
  float lumaB = dot(rgbB, luma);

  if((lumaB < lumaMin) || (lumaB > lumaMax)){
    gl_FragColor.xyz=rgbA;
  }else{
    gl_FragColor.xyz=rgbB;
  }
}
