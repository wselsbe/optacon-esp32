add_library(usermod_pz_actuator INTERFACE)

target_sources(usermod_pz_actuator INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pz_actuator.c
    ${CMAKE_CURRENT_LIST_DIR}/waveform.c
    ${CMAKE_CURRENT_LIST_DIR}/drv2665.c
    ${CMAKE_CURRENT_LIST_DIR}/shift_register.c
)

target_include_directories(usermod_pz_actuator INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_pz_actuator)
