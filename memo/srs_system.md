# component

* L0
    * CANDevices
        * WheelModule(+IMU)
        * GunModule
        * Target
        * RangeSencor
        * Light
    * USBDevices
        * camera
        * lidar
    * other
* L1
    * cart_navigator
        * in: move_target
        * out: twist
    * gun_navigator
        * in: head_target
        * out: twist
    * indicater
    * laser_pipeline
    * [object/obstacle]laser_detector
        * in: laser/odom
        * out: odstacle_distance/object_list
    * image_rect
    * [object]image_detector
        * in: image
        * out: roi
    * [object/localize]alver_map
* L2
    * guidance
    * [localize/obstacle]gmapping
        * in: laser/odom
        * out: map
    * [object]object_merger
    * [localize]landmark_localizer
* L3
    * swicher
    * manager
        * power
        * position
        * obserbe
        * connection
        * device

* ss (sub system)
    * logger
    * DB
    * (WebUi)
    * file_server
    * remote_server
    * parameter_sever
    * self_monitor

* other system
    * remote_app
    * s4_server

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

* user_input
* manager_input
* actor_input
* BT
* actor_output

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

# cart_navigator
* pligins
    * manual
    * twist
    * pose
    * trajectory
* command
    * mode
    * pose2d
    * trajectory
* state
    * mode
        * interrupted
        * blocked
        * no_input
    * state
        * pre_condition
        * active
        * goaled
    * distance

# omni_guidance
* plugins
    * manual
    * free_move
    * graph_move
    * replay
    * explore
    * patrol
    * follow

## swicher
* device_state
    * initial
    * localiza_error
    * power_error
    * observation_error
    * 

* input
  * knownledge_source base
    * object_event
    * device_status
* process
depend graph
  * proccess base
  * bt
  * goap
* output
  * effector base

## managed_item
* int id
* string name
* bool available
* bool active
* int[] option

* int status(NORMAL,WARNING,ERROR,NOT_ACTIVE,NOT_CONNECT)
* int reason
* int option

list
* booting
* power
* localize
* obserbe
* connection
* device

* diag
* knowledge_list



# repo 
## software
* srs004_ros
* srs004_android
* srs004_remote
* srs004_stm32
## hardware
* srs004_mech
* srs004_elec

## remote