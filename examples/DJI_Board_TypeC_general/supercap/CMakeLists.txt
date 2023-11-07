project(example_supercap_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET ${BOARD_NAME}
        SOURCES main.cpp)