PROJECT(MADGT9X)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../extrenlib/lib) 

include(./chal_export/flags.cmake)

SET(CMAKE_C_COMPILER arm-zephyr-eabi-gcc)

INCLUDE_DIRECTORIES(./)
INCLUDE_DIRECTORIES(./chal_export/include/)

ADD_COMPILE_OPTIONS(${C_FLAGS})

SET(LIBMADGT9X_SRC MadGt9x.c)
ADD_LIBRARY(madgt9x STATIC ${LIBMADGT9X_SRC})

add_custom_command(TARGET madgt9x POST_BUILD
	COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/../externlib/inc
    COMMAND cp ${PROJECT_SOURCE_DIR}/MadGt9x.h ${PROJECT_SOURCE_DIR}/../externlib/inc)