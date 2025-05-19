#cmake_minimum_required(VERSION 2.6)

include(GNUInstallDirs)

# tell cmake where to install the files
install(PROGRAMS
radioid_mqtt.py
DESTINATION bin)
