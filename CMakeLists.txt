cmake_minimum_required(VERSION 3.13.1)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(camera_app)

FILE(GLOB app_sources src/*.c)
FILE(GLOB video_sources src/video/*.c)

target_sources(app PRIVATE ${app_sources} ${video_sources} $ENV{ZEPHYR_BASE}/ext/hal/nxp/mcux/drivers/imx/fsl_csi.c)
