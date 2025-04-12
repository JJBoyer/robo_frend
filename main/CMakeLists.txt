idf_component_register(
	SRCS "state_estimation.cpp" "encoder.cpp" "pure_pursuit.cpp" "globals.cpp" "main.cpp" "motor_control.cpp" "sensor_control.cpp"
	"tasks.cpp" "telemetry.cpp" "task_manager.cpp" "imu.cpp" "globals.cpp" "path_planner.cpp" "encoder.cpp" "comms.cpp"
    INCLUDE_DIRS "." "../lib/eigen"
	REQUIRES cxx
	PRIV_REQUIRES driver esp_timer esp_system mpu6050 esp_wifi mqtt nvs_flash
	)
