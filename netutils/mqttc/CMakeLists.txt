if (CONFIG_NETUTILS_MQTTC)
  FetchContent_Declare(mqttc
    URL https://github.com/LiamBindle/MQTT-C/archive/${CONFIG_NETUTILS_MQTTC_VERSION}.tar.gz)
  FetchContent_Populate(mqttc)
  FetchContent_GetProperties(mqttc SOURCE_DIR MQTTC_SOURCE_DIR)

  file(GLOB SRCS LIST_DIRECTORIES false ${MQTTC_SOURCE_DIR})
  nuttx_add_user_library(mqttc ${SRCS})
  target_include_directories(mqtcc INTERFACE ${MQTTC_SOURCE_DIR}/include)
endif()
