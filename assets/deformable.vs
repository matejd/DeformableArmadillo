attribute vec4 baryCoords;
attribute vec4 tetraIndNor; // Tetra index (xy) + encoded normal (zw).

uniform mat4 mvp;
uniform mat4 lightMvp;
uniform mat4 Qs[400];
uniform mat3 tetrahedraITT[400];
uniform bool shadowGen;

varying vec3 vNormal;
varying vec3 vPos;
varying vec4 vProjPos;

void main() {
    int tetraIndex = int(tetraIndNor.x*256.0 + tetraIndNor.y);
    mat4 Q = Qs[tetraIndex];
    vec4 scaledBC = baryCoords * 5.0;
    vec3 position = scaledBC.x * Q[0].xyz +
                    scaledBC.y * Q[1].xyz +
                    scaledBC.z * Q[2].xyz +
                    scaledBC.w * Q[3].xyz;
    vPos = position;
    vProjPos = lightMvp * vec4(vPos, 1.0);
    if (shadowGen) {
        gl_Position = vProjPos;
    }
    else {
        // http://aras-p.info/texts/CompactNormalStorage.html
        // Lambert Azimuthal Equal-Area projection.
        vec2 fenc = tetraIndNor.zw*4.0/255.0 - 2.0;
        float f = dot(fenc, fenc);
        float g = sqrt(1.0 - f/4.0);
        vec3 nn;
        nn.xy = fenc*g;
        nn.z = f/2.0 - 1.0;
        vNormal = tetrahedraITT[tetraIndex] * normalize(nn);
        gl_Position = mvp * vec4(vPos, 1.0);
    }
}
