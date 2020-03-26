#ifdef ESP_BUILD_WITH_TRIANGLE_SENSOR
in int gl_PrimitiveID;
#endif

#ifdef ESP_BUILD_WITH_TRIANGLE_SENSOR
layout(location = 0) out int triangleId;
#endif

void main() {
    #ifdef ESP_BUILD_WITH_TRIANGLE_SENSOR
    triangleId = 7;
    #endif
}
