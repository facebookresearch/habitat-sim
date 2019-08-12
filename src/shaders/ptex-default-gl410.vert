layout(location = 0) in vec4 position;

uniform mat4 MVP;
uniform vec4 clipPlane;

void main()
{
    gl_ClipDistance[0] = dot(position, clipPlane);
    gl_Position = MVP * position;
}
