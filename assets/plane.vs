attribute vec2 position;
attribute vec2 uv;

uniform mat4 mvp;
uniform mat4 lightMvp;
uniform float groundY;

varying vec2 vUv;
varying vec3 vPos;
varying vec4 vProjPos; // Clip space (before division by w).

void main() {
    vUv = uv;
    vPos = vec3(position.x, groundY, position.y);
    vProjPos = lightMvp * vec4(vPos, 1.0);
    gl_Position = mvp * vec4(vPos, 1.0);
}
