add_library(usermod_pz_pwm INTERFACE)

target_sources(usermod_pz_pwm INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pz_pwm.c
)

target_include_directories(usermod_pz_pwm INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_pz_pwm)
