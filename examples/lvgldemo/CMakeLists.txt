############################################################################
# apps/examples/lvgdemo/Make.defs
# Adds selected applications to apps/ build
#
#   Copyright (C) 2018 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

if(CONFIG_EXAMPLES_LVGLDEMO)
  FetchContent_Declare(lvgldemo URL "https://github.com/lvgl/lv_examples/archive/v${CONFIG_LVGL_VERSION}.tar.gz")
  FetchContent_Populate(lvgldemo)
  FetchContent_GetProperties(lvgldemo)

  set(SRCS lvgldemo.c fbdev.c lcddev.c)

  if (CONFIG_INPUT)
    list(APPEND SRCS tp.c tp_cal.c)
  endif()

  # static common assets used in multiple examples

  set(LVGL_SRCS)
  file(GLOB LV_EXAMPLE_ASSETS
    RELATIVE ${lvgldemo_SOURCE_DIR}
    LIST_DIRECTORIES false
    ${lvgldemo_SOURCE_DIR}/lv_examples/assests/*.c)
  list(APPEND LVGL_SRCS ${LV_EXAMPLE_ASSETS})

  if (CONFIG_EXAMPLES_LVGLDEMO_BENCHMARK)
    list(APPEND LVGL_SRCS lv_demo_benchmark/lv_demo_benchmark.c)
  endif()

  if (CONFIG_EXAMPLES_LVGLDEMO_PRINTER)
    file(GLOB LV_PRINTER_IMAGES
      RELATIVE ${lvgldemo_SOURCE_DIR}
      LIST_DIRECTORIES false
      ${lvgldemo_SOURCE_DIR}/lv_examples/src/lv_demo_printer/images/*.c)

    list(APPEND LVGL_SRCS
      lv_demo_printer.c lv_demo_printer_theme.c ${LV_PRINTER_IMAGES})
  endif()

  if (CONFIG_EXAMPLES_LVGLDEMO_STRESS)
    list(APPEND LVGL_SRCS lv_demo_stress/lv_demo_stress.c)
  endif()

  if (CONFIG_EXAMPLES_LVGLDEMO_WIDGETS)
    list(APPEND LVGL_SRCS lv_demo_widgets/lv_demo_widgets.c)
  endif()

  list(TRANSFORM LVGL_SRCS PREPEND ${lvgldemo_SOURCE_DIR}/src/)

  nuttx_add_application(
    NAME lvgldemo
    PRIORITY ${CONFIG_EXAMPLES_LVGLDEMO_PRIORITY}
    STACKSIZE ${CONFIG_EXAMPLES_LVGLDEMO_STACKSIZE}
    SRCS ${SRCS} ${LVGL_SRCS})

  target_compile_options(apps_lvgldemo PRIVATE -Wno-format)
  target_link_libraries(apps_lvgldemo lvgl)
  target_include_directories(apps_lvgldemo PRIVATE ${CMAKE_CURRENT_LIST_DIR})
  target_compile_definitions(apps_lvgldemo PRIVATE
    LV_LVGL_H_INCLUDE_SIMPLE LV_EX_CONF_INCLUDE_SIMPLE)
endif()
