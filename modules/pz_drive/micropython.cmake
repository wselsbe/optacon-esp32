# modules/pz_drive/micropython.cmake
add_library(usermod_pz_drive INTERFACE)

target_sources(usermod_pz_drive INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pz_drive.c
    ${CMAKE_CURRENT_LIST_DIR}/hv509.c
    ${CMAKE_CURRENT_LIST_DIR}/drv2665.c
    ${CMAKE_CURRENT_LIST_DIR}/pwm.c
    ${CMAKE_CURRENT_LIST_DIR}/fifo.c
)

target_include_directories(usermod_pz_drive INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(usermod_pz_drive INTERFACE
    MICROPY_HW_ESP_NEW_I2C_DRIVER=1
)

target_link_libraries(usermod INTERFACE usermod_pz_drive)
