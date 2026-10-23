#version 330 core
out vec4 FragColor;

in vec3 ourColor;
in vec2 TexCoord;

uniform vec3 bullColor;
uniform sampler2D texture1;

void main()
{
    // Azul (bull, cow)
    if (distance(bullColor, vec3(0.0, 0.0, 1.0)) < 0.05) {
        FragColor = vec4(0.0, 0.0, 1.0, 1.0);
    }
    // Amarelo (tractor)
    else if (distance(bullColor, vec3(1.0, 1.0, 0.0)) < 0.05) {
        FragColor = vec4(1.0, 1.0, 0.0, 1.0);
    }
    // Rosa (pig)
    else if (distance(bullColor, vec3(1.0, 0.4, 0.7)) < 0.05) {
        FragColor = vec4(1.0, 0.4, 0.7, 1.0);
    }
    // Roxo (horse)
    else if (distance(bullColor, vec3(0.5, 0.0, 0.5)) < 0.05) {
        FragColor = vec4(0.5, 0.0, 0.5, 1.0);
    }
    // Marrom (deer)
    else if (distance(bullColor, vec3(0.4, 0.26, 0.13)) < 0.05) {
        FragColor = vec4(0.4, 0.26, 0.13, 1.0);
    }
    else {
        FragColor = texture(texture1, TexCoord);  // fallback
    }
}
