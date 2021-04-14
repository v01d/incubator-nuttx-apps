if (CONFIG_TESTING_LTP)
  message(FATAL_ERROR "ltp not supported yet")
#  FetchContent_Declare(ltp
#    GIT_REPOSITORY https://github.com/linux-test-project/ltp.git
#    PATCH_COMMAND patch -p0 ltp.patch)
#  FetchContent_Populate()
#  FetchContent_GetProperties(ltp SOURCE_DIR LTP_SOURCE_DIR)

#  set(BLACKWORDS
#    affinity.h endpwent langinfo.h ucontext.h noatime.h clock_getcpuclockid
#    CLOCK_PROCESS_CPUTIME_ID CLOCK_THREAD_CPUTIME_ID
#    fork() getpwent killpg pthread_atfork pthread_attr_setscope
#    pthread_condattr_setpshared pthread_condattr_getpshared
#    pthread_getattr_np pthread_getcpuclockid pthread_mutex_getprioceiling
#    pthread_mutexattr_getprioceiling pthread_mutexattr_setprioceiling
#    pthread_rwlockattr_destroy pthread_rwlockattr_getpshared
#    pthread_rwlockattr_init RLIMIT_MEMLOCK PTHREAD_SCOPE_PROCESS
#    setpwent SIGABRT SIGBUS SIGFPE SIGHUP SIGILL SIGPROF SIGSEGV SIGSYS
#    SIGTRAP SIGTSTP SIGTTIN SIGTTOU SIGURG SIGVTALRM SIGXCPU SIGXFSZ
#    SIGSTKSZ stack_t siginterrupt threads_scenarii.c
#    ILL_[A-Z] FPE_[A-Z] BUS_[A-Z] TRAP_[A-Z] SEGV_[A-Z] POLL_[A-Z]
#    SA_ONSTACK MINSIGSTKSZ FPE_FLTINV)

#  if (CONFIG_FS_AIO)
#    list(APPEND BLACKWORDS aio.h SIGPOLL)
#  endif()

#  set(BLACKSRCS
#    testfrmw.c 7-3-buildonly.c 7-4-buildonly.c
#    7-5-buildonly.c 11-1-buildonly.c 17-1-buildonly.c
#    19-1-buildonly.c 21-1-buildonly.c 22-31-buildonly.c
#    22-32-buildonly.c 25-1-buildonly.c 27-1-buildonly.c
#    35-1-buildonly.c 35-2-buildonly.c)

#  set(MAINWORDS "main(")

#  set(TESTDIR ${LTP_SOURCE_DIR}/testcases/open_posix_testsuite)

  #$(foreach word, $(BLACKWORDS), $(eval BLACKLIST+=$(shell find $(TESTDIR) -name *.c |xargs grep -lr $(word))))
  #$(foreach src, $(BLACKSRCS), $(eval BLACKLIST+=$(filter %$(src),$(shell find $(TESTDIR) -name *.c))))

  #LTP_ORIGINS  := $(filter-out $(BLACKLIST), $(shell find $(TESTDIR) -name *.c))

  #$(foreach word, $(MAINWORDS), $(eval LTP_MAINCSRCS+=$(shell grep -lr $(word) $(LTP_ORIGINS))))

  #LTP_CSRCS    := $(filter-out $(LTP_MAINCSRCS), $(LTP_ORIGINS))

  #PROGNAME     := $(basename $(shell echo $(LTP_MAINCSRCS) | xargs -n 1 | awk -F "[/]" '{print "ltp_"$$(NF-2)"_"$$(NF-1)"_"$$(NF)}' | sed s/-/_/g))
  #MAINSRC       = $(LTP_MAINCSRCS)
  #PRIORITY      = SCHED_PRIORITY_DEFAULT
  #STACKSIZE     = $(CONFIG_DEFAULT_TASK_STACKSIZE)
  #MODULE        = $(CONFIG_TESTING_LTP)

#  CSRCS        := $(LTP_CSRCS)
#  CFLAGS       += -I$(LTP_UNPACK)
#  CFLAGS       += -I$(TESTDIR)/include
#  endif

#  # Relax warning checks to avoid expected compile errors:
#  target_compile_options(libapps_ltp PRIVATE
#    -Wno-strict-prototypes -Wno-return-type -Wno-format -Wno-uninitialized
#    -Wno-unused-variable -Wno-unused-function -Wno-unused-but-set-variable
#    -Wno-unused-value
#    # Should be removed if possible in the future
#    -Wno-incompatible-pointer-types -Wno-overflow -Wno-int-to-pointer-cast)
endif()
