idf_component_register(
	SRCS "main.cpp" "pwm_control.cpp" "sensor_control.cpp" "tasks.cpp" "status.cpp" "task_manager.cpp"
    INCLUDE_DIRS "."
	REQUIRES cxx
	PRIV_REQUIRES esp_driver_gpio driver esp_adc esp_timer esp_system
	)
