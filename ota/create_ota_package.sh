#!/bin/bash
OTA_PACKAGE_NAME="ems"
OTA_VERSION=$(cat ${OTA_PACKAGE_NAME}.version)
OTA_DATE=$(date +%Y%m%d)
OTA_FILE_NAME="${OTA_PACKAGE_NAME}-${OTA_VERSION}-${OTA_DATE}.tgz"
STRATUX_PATH="/opt/stratux"
SYSTEMD_PATH="/lib/systemd/system"
RB_SETTINGS_PATH="/boot/firmware/rb"

# omitting /boot/firmware/rb/rb-vor.json until we have a way to update it without overwriting user settings
tar -czf ${OTA_FILE_NAME} \
     ${STRATUX_PATH}/bin/ems-adc \
     ${STRATUX_PATH}/bin/ems-chtegt \
     ${STRATUX_PATH}/bin/ems-gpio \
     ${STRATUX_PATH}/bin/ems-rpm \
     ${STRATUX_PATH}/bin/rb-sysmon \
     ${SYSTEMD_PATH}/ems-adc.service \
     ${SYSTEMD_PATH}/ems-chtegt.service \
     ${SYSTEMD_PATH}/ems-gpio.service \
     ${SYSTEMD_PATH}/ems-rpm.service \
     ${SYSTEMD_PATH}/rb-sysmon.service \
     ${STRATUX_PATH}/${OTA_PACKAGE_NAME}.version