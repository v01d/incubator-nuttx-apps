if (CONFIG_NETUTILS_CJSON)
  set(CJSON_VERSION 1.7.10)

  FetchContent_Declare(cjson URL https://github.com/DaveGamble/cJSON/archive)
  FetchContent_Populate(cjson)
  FetchContent_GetProperties(cjson SOURCE_DIR CJSON_SOURCE_DIR)

  nuttx_add_user_library(cjson
    ${CJSON_SOURCE_DIR}/cJSON.c
    ${CJSON_SOURCE_DIR}/cJSON_Utils.c)

  target_include_directories(cjson INTERFACE ${CJSON_SOURCE_DIR})
  target_compile_options(cjson PUBLIC CJSON_INCLUDE_CONFIG_H)
endif()
