############################################################################
# apps/examples/elf/Make.defs
# Adds selected applications to apps/ build
#
#   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

if(CONFIG_EXAMPLES_ELF)
  set(ENABLED_BINARIES)
  add_subdirectory(tests)

  set(BINARY_PATHS)
  foreach(binary ${ENABLED_BINARIES})
    list(APPEND BINARY_PATHS $<TARGET_PROPERTY:${binary},ELF_BINARY>)
  endforeach()

  if (CONFIG_EXAMPLES_ELF_ROMFS)
    nuttx_add_romfs(NAME elf
      FILES ${BINARY_PATHS}
      DEPENDS ${ENABLED_BINARIES})
  else()
    nuttx_add_cromfs(NAME elf
      FILES ${BINARY_PATHS}
      DEPENDS ${ENABLED_BINARIES})
  endif()

  nuttx_add_symtab(
    NAME elf
    BINARIES ${ENABLED_BINARIES}
    PREFIX g_elf)

	nuttx_add_application(
		NAME elf
		SRCS
      elf_main.c
      tests/dirlist.c
      $<$<BOOL:CONFIG_EXAMPLES_ELF_ROMFS>:romfs_elf.c>)

  target_include_directories(apps_elf PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/tests)
  target_link_libraries(apps PRIVATE symtab_elf)
endif()
