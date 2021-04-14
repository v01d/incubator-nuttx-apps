if (CONFIG_LIBUV)
  set(LIBUV_VERSION 1.38.1)

  FetchContent_Declare(libuv
    URL https://github.com/libuv/libuv/archive/v${LIBUV_VERSION}.zip
    PATCH_COMMAND patch -s -N -p1 0001-initial-libuv-port-to-nuttx.patch)
  FetchContent_Populate(libuv)
  FetchContent_GetProperties(libuv SOURCE_DIR LIBUV_SOURCE_DIR)

  add_subdirectory(libuv)
  add_subdirectory(test)
endif()
