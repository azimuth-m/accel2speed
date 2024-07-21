set(SOURCES spi_comms.cc my_gpio.cc)
idf_component_register(SRCS ${SOURCES}
                    REQUIRES driver
                    INCLUDE_DIRS . ../include )
