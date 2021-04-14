set(SRCS
  ${LIBUV_SOURCE_DIR}/src/unix/core.c
  ${LIBUV_SOURCE_DIR}/src/unix/poll.c
  ${LIBUV_SOURCE_DIR}/src/unix/loop.c
  ${LIBUV_SOURCE_DIR}/src/unix/thread.c
  ${LIBUV_SOURCE_DIR}/src/unix/no-proctitle.c)

if (CONFIG_DEV_URANDOM)
  list(APPEND SRCS
    ${LIBUV_SOURCE_DIR}/src/unix/random-devurandom.c
    ${LIBUV_SOURCE_DIR}/src/random.c)
endif()

if (CONFIG_LIBUV_LOOP_WATCHERS)
  list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/unix/loop-watcher.c)
endif()

# FIXME signal does not work yet
if (CONFIG_LIBUV_SIGNAL)
  list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/unix/signal.c)
endif()

# FIXME process does not work yet
if (CONFIG_LIBUV_PROCESS)
  list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/unix/process.c)
endif()

list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/unix/nuttx.c)

if (CONFIG_LIBUV_STREAM)
  list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/unix/nuttx_stream.c)
endif()

if (CONFIG_LIBUV_TCP)
  list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/unix/nuttx_tcp.c)
endif()

if (CONFIG_LIBUV_WQ)
  list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/unix/nuttx_threadpool.c)
endif()

if (CONFIG_LIBUV_ASYNC)
  list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/unix/async.c)
endif()

if (CONFIG_LIBUV_PIPE)
  list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/unix/pipe.c)
endif()

if (CONFIG_LIBUV_FS)
  list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/unix/fs.c)
endif()

if (CONFIG_LIBUV_FS_POLL)
  list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/fs-poll.c)
endif()

if (CONFIG_LIBUV_TIMER)
  if (CONFIG_LIBUV_TIMER_NUTTX)
    list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/unix/nuttx_timer.c)
  else()
    list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/timer.c)
  endif()
endif()

list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/uv-common.c)
list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/strscpy.c)

if (CONFIG_LIBUV_NET)
  list(APPEND SRCS ${LIBUV_SOURCE_DIR}/src/inet.c)
endif()

nuttx_add_user_library(libuv ${SRCS})
target_compile_options(libuv PUBLIC -D__NUTTX__)
target_include_directories(libuv PRIVATE
  ${LIBUV_SOURCE_DIR}/src
  ${LIBUV_SOURCE_DIR}/src/unix)
