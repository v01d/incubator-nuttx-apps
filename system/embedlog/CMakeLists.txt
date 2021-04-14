############################################################################
# apps/system/embedlog/Make.defs
#
#   Copyright (C) 2018 Michał Łyszczek. All rights reserved.
#   Author: Michał Łyszczek <michal.lyszczek@bofc.pl>
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

if(CONFIG_SYSTEM_EMBEDLOG)
  set(EMBEDLOG_VERSION 0.4.0)

  FetchContent_Declare(embedlog
    URL https://distfiles.kurwinet.pl/embedlog/embedlog-${EMBEDLOG_VERSION}.tar.gz)
  FetchContent_Populate(embedlog)
  FetchContent_GetProperties(embedlog SOURCE_DIR EMBEDLOG_SOURCE_DIR)

  nuttx_add_user_library(embedlog)

  set(SRCS
    ${EMBEDLOG_SOURCE_DIR}/src/el-options.c
    ${EMBEDLOG_SOURCE_DIR}/src/el-perror.c
    ${EMBEDLOG_SOURCE_DIR}/src/el-pmemory.c
    ${EMBEDLOG_SOURCE_DIR}/src/el-print.c
    ${EMBEDLOG_SOURCE_DIR}/src/el-puts.c
    ${EMBEDLOG_SOURCE_DIR}/src/el-ts.c)

  # compile-time configuration of embedlog

  if (CONFIG_EMBEDLOG_ENABLE_TIMESTAMP)
    target_compile_options(embedlog PRIVATE -DENABLE_TIMESTAMP)
    target_compile_options(embedlog PRIVATE -DENABLE_REALTIME)

    if (CONFIG_CLOCK_MONOTONIC)
      target_compile_options(embedlog PRIVATE -DENABLE_MONOTONIC)
    else()
      target_compile_options(embedlog PRIVATE -DENABLE_MONOTONIC=0)
    endif()
  endif()

  if (CONFIG_EMBEDLOG_ENABLE_FRACTIONS)
    target_compile_options(embedlog PRIVATE -DENABLE_FRACTIONS)
  else()
    target_compile_options(embedlog PRIVATE -DENABLE_FRACTIONS=0)
  endif()

  if (CONFIG_EMBEDLOG_ENABLE_COLORS)
    target_compile_options(embedlog PRIVATE -DENABLE_COLORS)
  else()
    target_compile_options(embedlog PRIVATE -DENABLE_COLORS=0)
  endif()

  if (CONFIG_EMBEDLOG_ENABLE_PREFIX)
    target_compile_options(embedlog PRIVATE -DENABLE_PREFIX)
    target_compile_options(embedlog PRIVATE -DEL_PREFIX_MAX=$(CONFIG_EMBEDLOG_PREFIX_MAX))
  else()
    target_compile_options(embedlog PRIVATE -DENABLE_PREFIX=0)
    target_compile_options(embedlog PRIVATE -DEL_PREFIX_MAX=0)
  endif()

  if (CONFIG_EMBEDLOG_ENABLE_FINFO)
    target_compile_options(embedlog PRIVATE -DENABLE_FINFO)
    target_compile_options(embedlog PRIVATE -DNOFINFO=0)
    target_compile_options(embedlog PRIVATE -DEL_FLEN_MAX=$(CONFIG_EMBEDLOG_FLEN_MAX))
  else()
    target_compile_options(embedlog PRIVATE -DENABLE_FINFO=0)
    target_compile_options(embedlog PRIVATE -DNOFINFO)
    target_compile_options(embedlog PRIVATE -DEL_FLEN_MAX=0)
  endif()

  if (CONFIG_EMBEDLOG_ENABLE_OUT_CUSTOM)
    target_compile_options(embedlog PRIVATE -DENABLE_OUT_CUSTOM)
  else()
    target_compile_options(embedlog PRIVATE -DENABLE_OUT_CUSTOM=0)
  endif()

  if (CONFIG_EMBEDLOG_ENABLE_OUT_STDERR)
    target_compile_options(embedlog PRIVATE -DENABLE_OUT_STDERR)
  else()
    target_compile_options(embedlog PRIVATE -DENABLE_OUT_STDERR=0)
  endif()

  if (CONFIG_EMBEDLOG_ENABLE_OUT_FILE)
    target_compile_options(embedlog PRIVATE -DENABLE_OUT_FILE)
    list(APPEND SRCS ${EMBEDLOG_SOURCE_DIR}/src/el-file.c)
  else()
    target_compile_options(embedlog PRIVATE -DENABLE_OUT_FILE=0)
  endif()

  if (CONFIG_EMBEDLOG_ENABLE_BINARY_LOGS)
    target_compile_options(embedlog PRIVATE -DENABLE_BINARY_LOGS)
    list(APPEND SRCS
      ${EMBEDLOG_SOURCE_DIR}/src/el-pbinary.c
      ${EMBEDLOG_SOURCE_DIR}/src/el-encode-number.c)
  else()
    target_compile_options(embedlog PRIVATE -DENABLE_BINARY_LOGS=0)
  endif()

  if (CONFIG_EMBEDLOG_ENABLE_OUT_TTY)
    target_compile_options(embedlog PRIVATE -DENABLE_OUT_TTY)
    list(APPEND SRCS ${EMBEDLOG_SOURCE_DIR}/src/el-tty.c)
  else()
    target_compile_options(embedlog PRIVATE -DENABLE_OUT_TTY=0)
  endif()

  if (CONFIG_SERIAL_TERMIOS)
    target_compile_options(embedlog PRIVATE -DHAVE_TERMIOS_H)
  else()
    target_compile_options(embedlog PRIVATE -DHAVE_TERMIOS_H=0)
  endif()

  target_compile_options(embedlog PRIVATE -DEL_LOG_MAX=$(CONFIG_EMBEDLOG_LOG_MAX))
  target_compile_options(embedlog PRIVATE -DEL_MEM_LINE_SIZE=$(CONFIG_EMBEDLOG_MEM_LINE_SIZE))

  # config.h is available only when building with autotools
  target_compile_options(embedlog PRIVATE -DHAVE_CONFIG_H=0)

  # embedlog uses access(path, F_OK) to determine if file exists or not and since
  # nuttx always returns OK here, we mark access as not working and make embedlog
  # to use stat() for that purpose
  target_compile_options(embedlog PRIVATE -DHAVE_ACCESS=0)

  # tell embedlog which features are available in nuttx os
  target_compile_options(embedlog PRIVATE -DENABLE_REENTRANT)
  target_compile_options(embedlog PRIVATE -DENABLE_OUT_SYSLOG)
  target_compile_options(embedlog PRIVATE -DHAVE_STAT)
  target_compile_options(embedlog PRIVATE -DHAVE_UNISTD_H)
  target_compile_options(embedlog PRIVATE -DHAVE_FSYNC)
  target_compile_options(embedlog PRIVATE -DHAVE_FILENO)
  target_compile_options(embedlog PRIVATE -DENABLE_COLORS_EXTENDED=0)

  # nuttx has its own snprintf implementation, disable internal snprintf
  # implementation
  target_compile_options(embedlog PRIVATE -DPREFER_PORTABLE_SNPRINTF=0)
  target_compile_options(embedlog PRIVATE -DNEED_SNPRINTF_ONLY=0)
  target_compile_options(embedlog PRIVATE -DHAVE_SNPRINTF)

  # not yet implemented features - silent compiler warnings
  target_compile_options(embedlog PRIVATE -DENABLE_OUT_NET=0)

  # nuttx does not implement clock() function
  target_compile_options(embedlog PRIVATE -DENABLE_CLOCK=0)

  target_sources(embedlog PRIVATE ${SRCS})

  target_include_directories(embedlog PUBLIC ${EMBEDLOG_SOURCE_DIR}/include)
endif()
