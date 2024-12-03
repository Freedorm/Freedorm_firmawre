idf_component_register(SRCS "ble_hidd_demo_main.c"
                            "esp_hidd_prf_api.c"
                            "hid_dev.c"
                            "hid_device_le_prf.c"
                            "../components/MultiButton/multi_button.c"  # 添加外部库源文件
                    INCLUDE_DIRS "."
                                 "../components/MultiButton")           # 添加外部库头文件目录

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
