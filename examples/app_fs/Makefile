# examples/app_fs/Makefile

# 이 Makefile 파일이 있는 폴더(= 앱 폴더)의 절대경로
APP_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

# Crazyflie 펌웨어 루트 (앱 폴더 기준 ../..)
CRAZYFLIE_BASE ?= $(abspath $(APP_DIR)/../..)

# 앱 레이어 설정 파일/루트
OOT_CONFIG ?= $(APP_DIR)/app-config
OOT ?= $(APP_DIR)

include $(CRAZYFLIE_BASE)/tools/make/oot.mk