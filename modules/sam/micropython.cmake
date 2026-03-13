# modules/sam/micropython.cmake — SAM speech synthesizer C module (s-macke/SAM)
add_library(usermod_sam INTERFACE)

target_sources(usermod_sam INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mod_sam.c
    ${CMAKE_CURRENT_LIST_DIR}/sam.c
    ${CMAKE_CURRENT_LIST_DIR}/render.c
    ${CMAKE_CURRENT_LIST_DIR}/reciter.c
    ${CMAKE_CURRENT_LIST_DIR}/debug.c
)

target_include_directories(usermod_sam INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../pz_drive
)

# Original SAM uses tentative definitions (A, X, Y in both sam.c and reciter.c)
target_compile_options(usermod_sam INTERFACE -fcommon)

target_link_libraries(usermod_sam INTERFACE usermod_pz_drive)
target_link_libraries(usermod INTERFACE usermod_sam)
