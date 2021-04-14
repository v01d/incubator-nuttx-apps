if (CONFIG_TESTING_UNITY)
  set(UNITY_VERSION 2.4.3)

  FetchContent_Declare(unity
    URL https://github.com/ThrowTheSwitch/Unity/archive/v${UNITY_VERSION}.tar.gz)
  FetchContent_Populate()
  FetchContent_GetProperties(unity SOURCE_DIR UNITY_SOURCE_DIR)

  nuttx_add_user_library(unity ${UNITY_SOURCE_DIR}/unity.c)

  target_compile_definitions(unity PRIVATE -DUNITY_INCLUDE_CONFIG_H)
  target_include_directories(unity INTERFACE ${UNITY_SOURCE_DIR})
endif()
