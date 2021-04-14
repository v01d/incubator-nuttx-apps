############################################################################
# apps/examples/tcpblaster/Make.defs
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

if(CONFIG_EXAMPLES_TCPBLASTER)
  set(SRCS tcpblaster_cmdline.c)

  if (CONFIG_EXAMPLES_TCPBLASTER_INIT)
    list(APPEND SRCS tcpblaster_netinit.c)
  endif()

  # Target 1 Files

  if (CONFIG_EXAMPLES_TCPBLASTER_LOOPBACK)
    list(APPEND SRCS tcpblaster_server.c tcpblaster_client.c)
  elseif (CONFIG_EXAMPLES_TCPBLASTER_SERVER)
    list(APPEND SRCS tcpblaster_server.c)
  else()
    list(APPEND SRCS tcpblaster_client.c)
  endif()

  # Target 1 Application

  nuttx_add_application(
    NAME ${CONFIG_EXAMPLES_TCPBLASTER_PROGNAME1}
    PRIORITY ${CONFIG_EXAMPLES_TCPBLASTER_PRIORITY1}
    STACKSIZE ${CONFIG_EXAMPLES_TCPBLASTER_STACKSIZE1}
    SRCS tcpblaster_target1.c ${SRCS})

  # Target 2 application

  if (CONFIG_EXAMPLES_TCPBLASTER_TARGET2)
    if (CONFIG_EXAMPLES_TCPBLASTER_SERVER)
      list(APPEND SRCS tcpblaster_client.c)
    else()
      list(APPEND SRCS tcpblaster_server.c)
    endif()

    nuttx_add_application(
      NAME ${CONFIG_EXAMPLES_TCPBLASTER_PROGNAME2}
      PRIORITY ${CONFIG_EXAMPLES_TCPBLASTER_PRIORITY2}
      STACKSIZE ${CONFIG_EXAMPLES_TCPBLASTER_STACKSIZE2}
      SRCS tcpblaster_target2.c ${SRCS})
  endif()
endif()
