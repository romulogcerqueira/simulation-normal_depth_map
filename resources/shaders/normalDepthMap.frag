#version 130

in vec3 pos;
in vec3 normal;
in float linearDepth;
uniform bool drawNormal;
uniform bool drawDepth;

void main() {
	vec4 tempInfo = vec4(0,0,0,0);
		
	if(drawNormal)
		if(linearDepth > 1){	
			tempInfo = vec4(-1.0,-1.0,-1.0,-1.0);			
		}else{ 
			tempInfo.zw = vec2(max(dot(normalize(-pos),normalize(normal)),0),1.0);			
		}		
				
	if(drawDepth)
		if(linearDepth > 1){	
			tempInfo = vec4(-1.0,-1.0,-1.0,-1.0);			
		}else{ 
			tempInfo.yw = vec2(1.0 - linearDepth, 1.0);			
		}
			
	gl_FragData[0] = tempInfo;
}