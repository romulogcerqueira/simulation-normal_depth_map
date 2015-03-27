#version 130

in vec3 pos;
in vec3 normal;
in float linearDepth;
uniform bool drawNormal;
uniform bool drawDepth;

void main() {
	gl_FragColor.rgba = vec4(0,0,0,0);
	
	if(drawNormal)
		gl_FragColor.ba = vec2(max(dot(normalize(-pos),normalize(normal)),0),1.0);		
	
	if(drawDepth)
		if(linearDepth > 1)	
			gl_FragColor = vec4(-1.0,-1.0,-1.0,-1.0);
		else 
			gl_FragColor.ga =vec2(1.0 - linearDepth, 1.0);
}