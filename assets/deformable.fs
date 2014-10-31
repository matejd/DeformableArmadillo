varying vec3 vNormal;
varying vec3 vPos;
varying vec4 vProjPos;

uniform vec3 lightPosition;
uniform vec3 camPosition;
uniform sampler2D shadowSampler;
uniform bool shadowGen;

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
    if (shadowGen) {
        gl_FragColor.r = gl_FragCoord.z; // In window-space (NDC + viewport transform). z is in range [0,1]. Linear depth maybe?
    }
    else {
        vec3 normal = normalize(vNormal);
        vec3 lightDir = normalize(lightPosition-vPos);
        vec3 viewDir = normalize(camPosition-vPos);
        // *Not* physically-based rendering!
        float diffuse = max(0.0, dot(normal, lightDir)); 
        float specular = pow(max(0.0, dot(reflect(-lightDir, normal), viewDir)), 64.0);
        vec3 ground = max(0.0, dot(normal, vec3(0.0, -1.0, 0.0))) * vec3(0.3, 0.3, 0.1);
        vec3 coords = (vProjPos.xyz / vProjPos.w)*0.5 + 0.5; // coords are in window-space.
        vec3 color = ground + (diffuse*vec3(1.0, 1.0, 0.0) + specular)*shadow(shadowSampler, coords.xy, coords.z);
        gl_FragColor.rgb = color;
    }
}
