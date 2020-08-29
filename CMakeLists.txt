PROJECT(MADGT9X)
include(./chal_export/flags.cmake)

SET(CMAKE_C_COMPILER arm-zephyr-eabi-gcc)

INCLUDE_DIRECTORIES(./)
INCLUDE_DIRECTORIES(./chal_export/include/)

ADD_COMPILE_OPTIONS(${C_FLAGS})

SET(LIBMADGT9X_SRC MadGt9x.c)
ADD_LIBRARY(madgt9x STATIC ${LIBMADGT9X_SRC})