# src/qmc_sensor/CMakeLists.txt

add_library(qmc_sensor_lib STATIC
        qmc5883l.cpp
        )

target_include_directories(qmc_sensor_lib PUBLIC 
        ${CMAKE_CURRENT_LIST_DIR}
        )

target_link_libraries(qmc_sensor_lib PUBLIC
        pico_stdlib
        hardware_i2c
        hardware_gpio
        )