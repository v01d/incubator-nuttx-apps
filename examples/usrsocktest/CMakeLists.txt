############################################################################
# examples/usrsocktest/Make.defs
# Adds selected applications to apps/ build
#
#   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

if(CONFIG_EXAMPLES_USRSOCKTEST)
	nuttx_add_application(
    PROGNAME ${CONFIG_EXAMPLES_USRSOCKTEST_PROGNAME}
    PRIORITY ${CONFIG_EXAMPLES_USRSOCKTEST_PRIORITY}
    STACKSIZE ${CONFIG_EXAMPLES_USRSOCKTEST_STACKSIZE}
    SRCS usrsocktest_main.c
      usrsocktest_daemon.c usrsocktest_basic_connect.c
      usrsocktest_basic_daemon.c usrsocktest_basic_getsockname.c
      usrsocktest_basic_getsockopt.c usrsocktest_basic_send.c
      usrsocktest_basic_setsockopt.c usrsocktest_block_recv.c
      usrsocktest_block_send.c usrsocktest_chardev.c
      usrsocktest_multi_thread.c usrsocktest_noblock_connect.c
      usrsocktest_noblock_recv.c usrsocktest_noblock_send.c
      usrsocktest_nodaemon.c usrsocktest_poll.c
      usrsocktest_remote_disconnect.c usrsocktest_wake_with_signal.c
  )
endif()
