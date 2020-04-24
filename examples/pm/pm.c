/****************************************************************************
 * pm/pm.c
 *
 *   Copyright (C) 2019 Matias Nitsche. All rights reserved.
 *   Author: Matias Nitsche <mnitsche@dc.uba.ar>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/boardctl.h>
#include <nuttx/timers/rtc.h>
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static void alarm_handler(int signo, FAR siginfo_t *info, FAR void *ucontext)
{
  UNUSED(signo);
  UNUSED(info);
  UNUSED(ucontext);
}

/****************************************************************************
 * pm_main
 ****************************************************************************/

int main(int argc, char *argv[])
{
  UNUSED(argc);
  UNUSED(argv);

  struct boardioc_pm_ctrl_s s;
  struct sigaction act;
  struct rtc_setperiodic_s periodic;
  sigset_t set;
  int fd;

  if (argc < 2)
    {
      fprintf(stderr, "Usage: pm <sleep seconds>\n");
      return ERROR;
    }

  printf("abcdefg\n");
  usleep(20000);

  sigemptyset(&set);
  sigaddset(&set, 1);
  sigprocmask(SIG_UNBLOCK, &set, NULL);

  /* Register alarm signal handler */

  act.sa_sigaction = alarm_handler;
  act.sa_flags     = SA_SIGINFO;

  sigfillset(&act.sa_mask);
  sigdelset(&act.sa_mask, 1);
  sigaction(1, &act, NULL);

  fd = open("/dev/rtc0", O_WRONLY);

  periodic.id = 0;
  periodic.pid = 0;
  periodic.event.sigev_signo = 1;
  periodic.event.sigev_notify = SIGEV_SIGNAL;
  periodic.period.tv_sec = atoi(argv[1]);
  periodic.period.tv_nsec = 0;

  ioctl(fd, RTC_SET_PERIODIC, (unsigned long)((uintptr_t)&periodic));

  s.domain = 0;
  s.state = PM_NORMAL;
  s.action = BOARDIOC_PM_RELAX;
  boardctl(BOARDIOC_PM_CONTROL, (uintptr_t)&s);
  s.state = PM_IDLE;
  s.action = BOARDIOC_PM_RELAX;
  boardctl(BOARDIOC_PM_CONTROL, (uintptr_t)&s);

  pause();

  printf("abcdefg\n");
  s.state = PM_NORMAL;
  s.action = BOARDIOC_PM_STAY;
  boardctl(BOARDIOC_PM_CONTROL, (uintptr_t)&s);
  s.state = PM_IDLE;
  s.action = BOARDIOC_PM_STAY;
  boardctl(BOARDIOC_PM_CONTROL, (uintptr_t)&s);
  return OK;
}
