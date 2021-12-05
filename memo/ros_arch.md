## architecture

```plantuml
component commander

package actor {
    component guidance
    component manual_distributer
    component move_navigator
    component illumination_navigator
}

package sensor {
    component localizer
    component gmapping
    component orb_slam
    component manager
    component perception
    component ar_alvar
}

package remote {
    component agent
}

package device {
    component ydlidar
    component realsense
    component mavros
}

package base {
    component mongodb
    component processor_status
    component data_server
}

commander --> guidance: guidance/target
guidance --> commander: guidance/status
guidance --> move_navigator: move_navigator/target
manual_distributer --> move_navigator: move_navigator/setpoint
guidance --> illumination_navigator: illumination_navigator/target

move_navigator --> mavros: local_target_ned
mavros --> perception: pose,twist
perception --> move_navigator: localization/local_odom
ydlidar --> move_navigator: laser_scan


commander --> manager: manager/target
manager --> commander: manager/check_list
manager --> guidance: manager/check_list
manager --> move_navigator: manager/check_list

```