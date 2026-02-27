add_library(usermod_pz_actuator INTERFACE)

target_sources(usermod_pz_actuator INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pz_actuator.c
    ${CMAKE_CURRENT_LIST_DIR}/drv2665.c
    ${CMAKE_CURRENT_LIST_DIR}/shift_register.c
    ${CMAKE_CURRENT_LIST_DIR}/task.c
    ${CMAKE_CURRENT_LIST_DIR}/sine.c
    ${CMAKE_CURRENT_LIST_DIR}/pwm.c
)

target_include_directories(usermod_pz_actuator INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(usermod_pz_actuator INTERFACE
    MICROPY_HW_ESP_NEW_I2C_DRIVER=1
)

target_link_libraries(usermod INTERFACE usermod_pz_actuator)
