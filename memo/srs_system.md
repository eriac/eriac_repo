# component

* L0
    * CANDevices
        * WheelModule
        * GunModule
        * Target
    * USBDevices
        * camera
        * lidar
    * other
* L1
    * controller+perception
        * cart+imu
        * gun(carrige+shooter+laser)
        * camera
        * other
* L2
    * navigator
    * recognition
* L3
    * swicher
    * manager
        * power
        * position
        * obserbe
        * connection
        * device
* sub system
    * logger
    * ftp
    * DB
    * WebUi
    * indicator
* L4
    * remote_app

# connection

```plantuml
a->b
```

# fuction
* mode = commander mode
    * [1]remote_mode
    * [n]navigator_mode
        * [n]controller_mode
    * [n]recognition_mode

# name 
* perception
    * perception
    * recognision
* control
    * control
    * navigation
* back

# switcher
* device_state
* user_state
* output_converter

# navigator
* result
    * SUCCESS
    * ABORT
    * PREEMPTED 
    * int option = 0;

* data
    * manual
    * position
        * graph
        * odom
    * obserbe
        * landmark
        * object
        * map
    * time
        * now
        * dt
    * manager_status

* void Initial(DATA, PARAM)
* STATE transition(DATA, PARAM)
* void update(DATA, PARAM)
* OUTPUT action(DATA, PARAM)



# controller
* void Initial(DATA, PARAM)
* OUTPUT action(DATA, PARAM)
