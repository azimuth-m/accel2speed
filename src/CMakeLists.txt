set(SOURCES spi_comms.cc InputGpio.cc OutputGpio.cc)
idf_component_register(SRCS ${SOURCES}
                    REQUIRES driver esp_event
                    INCLUDE_DIRS . ../include )
