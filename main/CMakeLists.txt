idf_component_register(
	SRCS "globals.cpp" "main.cpp" "motor_control.cpp" "sensor_control.cpp"
	"tasks.cpp" "status.cpp" "task_manager.cpp" "imu.cpp" "globals.cpp"
    INCLUDE_DIRS "."
	REQUIRES cxx
	PRIV_REQUIRES driver esp_timer esp_system mpu6050
	)
