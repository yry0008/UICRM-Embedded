
project(Doorbell ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_sender
        TARGET F103_Nano_general
        SOURCES sender.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_receiver
        TARGET F103_Nano_general
        SOURCES receiver.cpp)
