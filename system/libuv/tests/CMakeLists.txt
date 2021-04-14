if (CONFIG_LIBUV_UNIT_TESTS)
  set(SRCS
    test_main.c
    runner-nuttx.c
    ${LIBUV_SOURCE_DIR}/test/runner.c

    # Tests
    ${LIBUV_SOURCE_DIR}/test/test-loop-time.c
    ${LIBUV_SOURCE_DIR}/test/test-async.c
    ${LIBUV_SOURCE_DIR}/test/test-ip4-addr.c
    ${LIBUV_SOURCE_DIR}/test/test-active.c
    ${LIBUV_SOURCE_DIR}/test/test-idle.c
    ${LIBUV_SOURCE_DIR}/test/test-timer.c
    ${LIBUV_SOURCE_DIR}/test/test-timer-from-check.c
    ${LIBUV_SOURCE_DIR}/test/test-timer-again.c
    ${LIBUV_SOURCE_DIR}/test/test-threadpool.c
    ${LIBUV_SOURCE_DIR}/test/test-walk-handles.c
    ${LIBUV_SOURCE_DIR}/test/test-poll-close.c
    ${LIBUV_SOURCE_DIR}/test/test-loop-close.c
    ${LIBUV_SOURCE_DIR}/test/test-loop-stop.c
    ${LIBUV_SOURCE_DIR}/test/test-tcp-read-stop.c
    ${LIBUV_SOURCE_DIR}/test/test-ping-pong.c
    ${LIBUV_SOURCE_DIR}/test/test-tcp-write-after-connect.c
    ${LIBUV_SOURCE_DIR}/test/test-fs-copyfile.c
    ${LIBUV_SOURCE_DIR}/test/test-fs-poll.c)

  if (CONFIG_DEV_URANDOM)
    list(APPEND SRCS ${LIBUV_SOURCE_DIR}/test/test-random.c)
  endif()

  # Test helpers
  list(APPEND SRCS ${LIBUV_SOURCE_DIR}/test/echo-server.c)

  # libuv unit-tests command
  nuttx_add_application(
    NAME ${CONFIG_LIBUV_UNIT_TESTS_PROGNAME}
    PRIORITY ${CONFIG_LIBUV_UNIT_TESTS_PRIORITY}
    STACKSIZE ${CONFIG_LIBUV_UNIT_TESTS_STACKSIZE}
    SRCS ${SRCS})

  # Compilation flags
  target_link_libraries(libapps_${CONFIG_LIBUV_UNIT_TESTS_PROGNAME} PRIVATE libuv)
  target_include_directories(libapps_${CONFIG_LIBUV_UNIT_TESTS_PROGNAME} ${LIBUV_SOURCE_DIR}/test ${CMAKE_CURRENT_LIST_DIR})
endif()
