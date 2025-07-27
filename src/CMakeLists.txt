# src/CMakeLists.txt

add_subdirectory(qmc_sensor)

add_executable(${ProjectName}
    main.cpp
)

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/qmc_sensor
)

target_link_libraries(${ProjectName} 
    pico_stdlib 
    hardware_gpio
    FreeRTOS-Kernel-Heap4
    qmc_sensor_lib 
    hardware_i2c
    )

pico_add_extra_outputs(${ProjectName})
