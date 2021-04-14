if (CONFIG_INTERPRETERS_QUICKJS)
  set(QUICKJS_VERSION 2020-11-08)

  FetchContent_Declare(quickjs
    URL https://bellard.org/quickjs/quickjs-${QUICKJS_VERSION}.tar.xz
    PATCH_COMMAND patch -p1 < 0001-Disabled-unsupported-feature-on-NuttX.patch)
  FetchContent_Populate(quickjs)
  FetchContent_GetProperties(quickjs SOURCE_DIR QUICKJS_SOURCE_DIR)

  set(SRCS)
  if (CONFIG_INTERPRETERS_QUICKJS_MINI)
    list(APPEND SRCS qjsmini.c)
  else()
    list(APPEND SRCS qjs.c quickjs-libc.c repl.c)
  endif()

  list(APPEND SRCS quickjs.c libregexp.c libbf.c libunicode.c cutils.c)

  set(QUICKJS_FLAGS
    -DCONFIG_VERSION=${QUICKJS_VERSION} -Wno-shadow
    -Wno-array-bounds -I$(QUICKJS_UNPACK)
    -D__linux__ -include alloca.h
    -Wno-incompatible-pointer-types
    -Wno-implicit-function-declaration
    -Wno-unused-function
    -Wno-format)

  if (CONFIG_ARCH_ARM)
    list(APPEND QUICKJS_FLAGS
      -DFE_TONEAREST=0x00000000
      -DFE_UPWARD=0x00400000
      -DFE_DOWNWARD=0x00800000
      -DFE_TOWARDZERO=0x00c00000)
  endif()

  if (CONFIG_INTERPRETERS_QUICKJS_BIGNUM)
    list(APPEND QUICKJS_FLAGS -DCONFIG_BIGNUM)
    list(APPEND_SRCS qjscalc.c)
  endif()

  list(TRANSFORM SRCS PREPEND ${QUICKJS_SOURCE_DIR}/)
  nuttx_add_application(
    NAME qjs
    PRIORITY ${CONFIG_INTERPRETERS_QUICKJS_PRIORITY}
    STACKSIZE ${CONFIG_INTERPRETERS_QUICKJS_STACKSIZE}
    SRCS ${SRCS})

  target_compile_options(libapps_qjs PRIVATE ${QUICKJS_FLAGS})
endif()
