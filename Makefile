ROS_DISTRO ?= jazzy
MROS_WS ?= $(HOME)/mros/mros_ws
SERIAL_DEV ?= /dev/ttyUSB0
SERIAL_BAUD ?= 921600
IMU_SERIAL_DEV ?= $(SERIAL_DEV)
IMU_SERIAL_BAUD ?= $(SERIAL_BAUD)
IMU_POSE ?= unknown
SHELL := /bin/bash

CLEAN_ROS_ENV := unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH \
	GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH GAZEBO_MODEL_PATH GAZEBO_RESOURCE_PATH ROS_PACKAGE_PATH &&
ROS_SETUP := $(CLEAN_ROS_ENV) source /opt/ros/$(ROS_DISTRO)/setup.bash && source install/setup.bash
MROS_SETUP := $(CLEAN_ROS_ENV) source /opt/ros/$(ROS_DISTRO)/setup.bash && source $(MROS_WS)/install/setup.bash
REAL_ROS_SETUP := $(CLEAN_ROS_ENV) source /opt/ros/$(ROS_DISTRO)/setup.bash && source install/setup.bash && source $(MROS_WS)/install/setup.bash

.PHONY: help build gui real real-arm real-no-controller rz-real agent imu-test imu-monitor imu-measure firmware firmware-test microros-lib firmware-flash firmware-clean clean

help:
	@echo "megaDog real-hardware helpers:"
	@echo "  make build              - build ROS2 workspace"
	@echo "  make gui                - open megaDog Tkinter control panel"
	@echo "  make agent              - run micro_ros_agent serial bridge"
	@echo "  make real               - real hardware launch, safe mode: no megadog_controller"
	@echo "  make real-arm           - real hardware launch + start megadog_controller"
	@echo "  make rz-real            - RViz real view on ROS_DOMAIN_ID=0, fixed frame base"
	@echo "  make imu-test           - IMU only: agent + Kalman + monitor, no controller"
	@echo "  make imu-monitor        - open read-only IMU monitor for an existing pipeline"
	@echo "  make imu-measure        - record /imu/data balance JSON, set IMU_POSE=home/stand"
	@echo "  make firmware-test      - host protocol test, no hardware touched"
	@echo "  make microros-lib       - regenerate firmware micro-ROS lib/type-support"
	@echo "  make firmware           - build STM32H7 firmware"
	@echo "  make firmware-flash     - build + flash STM32H7 via ST-Link"

build:
	$(CLEAN_ROS_ENV) source /opt/ros/$(ROS_DISTRO)/setup.bash && colcon build --symlink-install

gui:
	$(REAL_ROS_SETUP) && ROS_DOMAIN_ID=0 ros2 run gui gui

agent:
	$(REAL_ROS_SETUP) && ROS_DOMAIN_ID=0 ros2 run micro_ros_agent micro_ros_agent serial --dev $(SERIAL_DEV) -b $(SERIAL_BAUD)

real:
	$(REAL_ROS_SETUP) && ros2 launch megadog_description real_ros2_control.launch.py \
		serial_dev:=$(SERIAL_DEV) serial_baud:=$(SERIAL_BAUD) start_controller:=false

real-no-controller: real

real-arm:
	$(REAL_ROS_SETUP) && ros2 launch megadog_description real_ros2_control.launch.py \
		serial_dev:=$(SERIAL_DEV) serial_baud:=$(SERIAL_BAUD) start_controller:=true

rz-real:
	$(ROS_SETUP) && ros2 launch megadog_description rz_real.launch.py

imu-test:
	$(REAL_ROS_SETUP) && ROS_DOMAIN_ID=0 ros2 launch megadog_description imu_test.launch.py \
		serial_dev:=$(IMU_SERIAL_DEV) serial_baud:=$(IMU_SERIAL_BAUD)

imu-monitor:
	$(REAL_ROS_SETUP) && ROS_DOMAIN_ID=0 ros2 run gui imu_monitor

imu-measure:
	$(REAL_ROS_SETUP) && ROS_DOMAIN_ID=0 IMU_POSE=$(IMU_POSE) ros2 run gui imu_measure

firmware:
	$(MAKE) -C firmware/stm32h7

firmware-test:
	$(MAKE) -C firmware/stm32h7 test

microros-lib:
	cd firmware/stm32h7/microros && $(MROS_SETUP) && \
		ros2 run micro_ros_setup build_firmware.sh -- \
		$(CURDIR)/firmware/stm32h7/microros/toolchain.cmake \
		$(CURDIR)/firmware/stm32h7/microros/firmware/mcu_ws/colcon.meta

firmware-flash:
	$(MAKE) -C firmware/stm32h7 flash

firmware-clean:
	$(MAKE) -C firmware/stm32h7 clean

clean:
	rm -rf build install log
