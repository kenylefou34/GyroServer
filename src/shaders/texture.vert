#version 330
in vec3 vertexPosition;
in vec2 vertexUV;
out vec2 fragUV;

uniform mat4 mvp;

void main() {
    fragUV = vertexUV;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
