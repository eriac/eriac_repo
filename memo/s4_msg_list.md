# device
* common
  * diag
  * debug_value
* [01]ShieldBoard
* [02]PowerBoard
  * power_status
* [03]WheelBoard
  * twist
  * odom2D
  * motor_status
* [04]GunBoard
  * shot/command
  * laser/command
  * laser/status
  * turret/aim
  * turret/attitude
* [05]UiBoard
  * set_led
  * buttons
* [06]LineSensorBoard
  * line[9]
* [07]SonorBoard
  * range[2] 
* [08~10]TargetBoard_x3
  * set_led
  * buttons
* [11]LightBoard
  * set_led

# sensor
* exlorer
* 2d_object
* waypoint
* graph

# remote
## midleware
* command(ordered, random)
* request(ordered, random)
* setpoint
* telemetry
* event
* param
* data

## usecase
* command
  * item_id
  * param
* request
* data
* setpoint
  * joy
* telemetry
  * status
    * vehicle_status
    * mode_request
    * action_request
  * switcher_state
    * bt_id
    * sub_id
* event
  * event_id
  * from_id
  * to_id
  * source
  * source_id
* param/feedback
  * item
    * name
    * last_action: INITIAL,CHANGING,CHANGED
    * last_stamp
    * last_source



