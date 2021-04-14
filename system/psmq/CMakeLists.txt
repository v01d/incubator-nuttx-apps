if (CONFIG_SYSTEM_PSMQ)
  set(PSMQ_VERSION 0.1.0)

  FetchContent_Declare(psmq
    URL https://distfiles.kurwinet.pl/psmq/psmq-${PSMQ_VERSION}.tar.gz)
  FetchContent_Populate(psmq)
  FetchContent_GetProperties(psmq SOURCE_DIR PSMQ_SOURCE_DIR)

  nuttx_add_user_library(psmq)

  # mandatory source files to compile
  set(SRCS
    ${PSMQ_SOURCE_DIR}/lib/psmq.c
    ${PSMQ_SOURCE_DIR}/src/broker.c
    ${PSMQ_SOURCE_DIR}/src/cfg.c
    ${PSMQ_SOURCE_DIR}/src/globals.c
    ${PSMQ_SOURCE_DIR}/src/topic-list.c)

  target_compile_options(psmq PRIVATE
    # compile-time options from Kconfig
    -DPSMQ_MAX_CLIENTS=${CONFIG_PSMQ_MAX_CLIENTS}
    -DPSMQ_PAYLOAD_MAX=${CONFIG_PSMQ_PAYLOAD_MAX}
    -DPSMQ_TOPIC_MAX=${CONFIG_PSMQ_TOPIC_MAX}

    # build psmq as library and disable standalone applications (apps will
    # still be compiled but into shared library rather than standalone application
    -DPSMQ_LIBRARY=1
    -DPSMQ_STANDALONE=0

    # psmq cannot be damonized in nuttx
    -DPSMQ_ENABLE_DAEMON=0

    # nuttx has no opterr variable
    -DPSMQ_NO_OPTERR=1

    # nuttx does not really support signals, so disable them. There will be no way
    # to gently kill programs.
    -DPSMQ_NO_SIGNALS=1

    # package string needed by tools when passing '-v' option
    -DPACKAGE_STRING='psmq ${PSMQ_VERSION}'
    -DPACKAGE_VERSION='${PSMQ_VERSION}')

  # register psmqd broker daemon application
  nuttx_add_application(
    NAME psmqd
    PRIORITY ${CONFIG_PSMQD_PRIORITY}
    STACKSIZE ${CONFIG_PSMQD_STACKSIZE}
    SRCS ${PSMQ_SOURCE_DIR}/src/psmqd.c)

  # register psmq_pub application if it was enabled
  if (CONFIG_PSMQ_TOOLS_PUB)
    nuttx_add_application(
      NAME psmq_pub
      PRIORITY ${CONFIG_PSMQ_PUB_PRIORITY}
      STACKSIZE ${CONFIG_PSMQ_PUB_STACKSIZE}
      SRCS ${PSMQ_SOURCE_DIR}/src/psmq-pub.c
      DEPENDS psmq)
  endif()

  # register psmq_sub application if it was enabled
  if (CONFIG_PSMQ_TOOLS_SUB)
    nuttx_add_application(
      NAME psmq_sub
      PRIORITY ${CONFIG_PSMQ_SUB_PRIORITY}
      STACKSIZE ${CONFIG_PSMQ_SUB_STACKSIZE}
      SRCS ${PSMQ_SOURCE_DIR}/src/psmq-sub.c
      DEPENDS psmq)
  endif()

  target_include_directories(psmq PRIVATE ${PSMQ_SOURCE_DIR})
  target_include_directories(psmq INTERFACE ${PSMQ_SOURCE_DIR}/inc)
endif()
