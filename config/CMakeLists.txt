#cmake_minimum_required(VERSION 2.6)

include(GNUInstallDirs)

# tell cmake where to install the files
install(FILES 
indiserver.service
radioid_mqtt.service
DESTINATION /usr/lib/systemd/system)

install(CODE "execute_process(COMMAND systemctl daemon-reload)")
