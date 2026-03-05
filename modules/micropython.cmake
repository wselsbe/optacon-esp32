include(${CMAKE_CURRENT_LIST_DIR}/pz_drive/micropython.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/board_utils/micropython.cmake)

# Disable BLE to save flash space (~184 KB); we only use WiFi
list(APPEND MICROPY_CPP_DEF_EXTRA "MICROPY_PY_BLUETOOTH=0")
add_compile_definitions(MICROPY_PY_BLUETOOTH=0)

