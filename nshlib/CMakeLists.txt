############################################################################
# apps/nshlib/CMakeLists.txt
#
#   Copyright (C) 2011-2013, 2016 Gregory Nutt. All rights reserved.
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

if(CONFIG_NSH_LIBRARY)
	set(SRCS
    nsh_init.c nsh_parse.c nsh_console.c nsh_script.c nsh_system.c
    nsh_command.c nsh_fscmds.c nsh_ddcmd.c nsh_proccmds.c nsh_mmcmds.c
    nsh_timcmds.c nsh_envcmds.c nsh_syscmds.c nsh_dbgcmds.c nsh_fsutils.c)

  if (CONFIG_FILE_STREAM)
    list(APPEND SRCS nsh_session.c)

    if (CONFIG_NSH_CONSOLE_LOGIN)
      list(APPEND SRCS nsh_login.c)
    endif()
  else()
    list(APPEND SRCS nsh_stdsession.c)

    if (CONFIG_NSH_CONSOLE_LOGIN)
      list(APPEND SRCS nsh_stdlogin.c)
    endif()
  endif()

  if (CONFIG_NSH_BUILTIN_APPS)
    list(APPEND SRCS nsh_builtin.c)
  endif()

  if (CONFIG_NSH_FILE_APPS)
    list(APPEND SRCS nsh_fileapps.c)
  endif()

  if (CONFIG_NSH_VARS)
    list(APPEND SRCS nsh_vars.c)
  endif()

  if (CONFIG_NSH_ROMFSETC)
    list(APPEND SRCS nsh_romfsetc.c)
  endif()

  if (CONFIG_NET)
    list(APPEND SRCS nsh_netcmds.c)

    if (CONFIG_NET_ROUTE)
      list(APPEND SRCS nsh_routecmds.c)
    endif()
  endif()

  if (NOT CONFIG_DISABLE_MOUNTPOINT)
    list(APPEND SRCS nsh_mntcmds.c)
  endif()

  if (CONFIG_MODULE)
    if (NOT CONFIG_NSH_DISABLE_MODCMDS)
      list(APPEND SRCS nsh_modcmds.c)
    endif()
  endif()

  if (CONFIG_NSH_CONSOLE)
    list(APPEND SRCS nsh_consolemain.c)
  endif()

  if (NOT CONFIG_NSH_DISABLE_PRINTF)
    list(APPEND SRCS nsh_printf.c)
  endif()

  if (CONFIG_NSH_TELNET)
    list(APPEND SRCS nsh_telnetd.c)

    if (CONFIG_NSH_TELNET_LOGIN)
      list(APPEND SRCS nsh_telnetlogin.c)
    endif()
  endif()

  if (NOT CONFIG_NSH_DISABLESCRIPT)
    list(APPEND SRCS nsh_test.c)
  endif()

  if (CONFIG_USBDEV)
    list(APPEND SRCS nsh_usbconsole.c)
  endif()

  if (CONFIG_NSH_ALTCONDEV)
    list(APPEND SRCS nsh_altconsole.c)
  endif()

  if (CONFIG_NSH_USBDEV_TRACE)
    list(APPEND SRCS nsh_usbtrace.c)
  endif()

  if (CONFIG_NETUTILS_CODECS)
    list(APPEND SRCS nsh_codeccmd.c)
  endif()

  if (CONFIG_NSH_LOGIN_PASSWD)
    list(APPEND SRCS nsh_passwdcmds.c)
  endif()

  nuttx_add_user_library(nshlib ${SRCS})
endif()
