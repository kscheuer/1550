cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME 1550)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    App/Src/thermal_control.c
    App/Src/laser_current.c
    App/Src/usb_comm.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Add post-build command to print size
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND arm-none-eabi-size $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Firmware Size:"
)

# --- USER CODE BEGIN Automation ---
# This creates a "flash" command for your terminal by adding a flash target that uses the STM32_Programmer_CLI to flash the generated .elf file to your hardware. Make sure to have the STM32_Programmer_CLI installed and in your system's PATH for this to work.
find_program(STM32_PROG_EXE STM32_Programmer_CLI)

if(STM32_PROG_EXE)
    add_custom_target(flash
        COMMAND ${STM32_PROG_EXE} -c port=SWD mode=Normal -w $<TARGET_FILE:${CMAKE_PROJECT_NAME}> -v -rst
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMENT "Flashing ${CMAKE_PROJECT_NAME}.elf to hardware..."
    )
else()
    message(WARNING "STM32_Programmer_CLI not found. Flash target will not work.")
endif()
# --- USER CODE END Automation ---

# --- USER CODE BEGIN USB-TERMINAL ---
set(COM_PORT "COM10")
set(BAUD_RATE "115200")

# Target for just the monitor
add_custom_target(monitor
    COMMAND python -m serial.tools.miniterm ${COM_PORT} ${BAUD_RATE} --raw
    COMMENT "Opening miniterm in VS Code (No Echo)..."
    USES_TERMINAL
)

# Target for the full Build -> Flash -> Monitor flow
add_custom_target(run
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target flash
    COMMAND python -c "import time; time.sleep(4)"
    COMMAND python -m serial.tools.miniterm ${COM_PORT} ${BAUD_RATE} --raw
    COMMENT "Building, Flashing, and Monitoring (No Echo)..."
    USES_TERMINAL
)

# Enable printf float support
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

# --- USER CODE END USB-TERMINAL ---

