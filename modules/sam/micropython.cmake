# modules/sam/micropython.cmake — SAM speech synthesizer C module
add_library(usermod_sam INTERFACE)

target_sources(usermod_sam INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mod_sam.c
    ${CMAKE_CURRENT_LIST_DIR}/sam.c
    ${CMAKE_CURRENT_LIST_DIR}/render.c
    ${CMAKE_CURRENT_LIST_DIR}/reciter.c
)

target_include_directories(usermod_sam INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../pz_drive
)

# SAM uses duplicate globals (A, X, Y) across translation units
target_compile_options(usermod_sam INTERFACE -fcommon)

target_link_libraries(usermod_sam INTERFACE usermod_pz_drive)
target_link_libraries(usermod INTERFACE usermod_sam)
