uniform vec3 lightPosition;
uniform vec3 lightDirection;
uniform float lightInnerCosAngle;
uniform float lightOuterCosAngle;
uniform sampler2D shadowSampler;

varying vec2 vUv;
varying vec3 vPos;
varying vec4 vProjPos;

const float shadowTexSize = 1024.0;

float shadow(sampler2D ss, vec2 uv, float depth)
{
    // Poisson 4-tap PCF.
    float p0 = step(depth, texture2D(ss, uv + vec2(-0.94201624,-0.3990621)/shadowTexSize).r);
    float p1 = step(depth, texture2D(ss, uv + vec2( 0.94558609,-0.7689072)/shadowTexSize).r);
    float p2 = step(depth, texture2D(ss, uv + vec2(-0.09418410,-0.9293887)/shadowTexSize).r);
    float p3 = step(depth, texture2D(ss, uv + vec2( 0.34495938, 0.2938776)/shadowTexSize).r);
    return (p0+p1+p2+p3)*(1.0/4.0);
}

void main() {
    vec2 uv = fract(vUv*20.0);
    float width = 0.03;
    vec3 color = vec3(smoothstep(0.0, width, min(uv.x, uv.y)) * // Reduce frequency content => less aliasing.
                      (1.0-smoothstep(1.0-width, 1.0, max(uv.x, uv.y))));
    float lightDot = dot(normalize(vPos-lightPosition), lightDirection);
    color *= smoothstep(lightOuterCosAngle, lightInnerCosAngle, lightDot);

    vec3 coords = (vProjPos.xyz / vProjPos.w)*0.5 + 0.5; // coords are in window-space.
    gl_FragColor.rgb = color * shadow(shadowSampler, coords.xy, coords.z);
}
