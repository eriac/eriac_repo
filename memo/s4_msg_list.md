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

## remote access resource

* command
  * [put_c] set command
  * [get_c] get command list
* notiry
  * status
    * [get_c]
  * event
    * [get_c]
* parameter
  * [get_c] get pages
  * {paga name}
  * [get_c] get list in page
    * {param name}
      * [get_c] get parameter
      * [put_c] set parameter  

* telemetry
  * [get_c] get telemetry list
  * {name}
    * [get_c] get data
* setpoint
  * [get_c] get list
  * {name}
    * [put_c] send stream
* data
  * [get_c] get list
  * {name}
    * [request] request data
    * [get_s] get data
    * [put_s] send data

* system
  * vehicle_status
  * ping-pong
  * capacity

### action
#### msg
* a->c index[]
  * name
  * seq
  * description
* a->c status[]
  * name
  * state
  * option[]
    * name
    * availability
    * next_state
    * reason[]
* c->a request
  * name
  * client_uuid
  * command_uuid
  * command
  * parameter
* a->c response
  * name
  * seq
  * status

#### API
* (get) GetHeaders 
* (get) GetDetail
* (Post) SendRequest
* (get) GetResponse

### notify

#### msg
* a->c index[]
  * name
  * seq
  * description
* a->c contents[]
  * name
  * seq
  * content

#### API
* (get)GetHeaders
  * param: none
  * return: index[]
    * name
    * status
    * description
* (get)GetDetail
  * param: page name
  * return: content[]
    * seq
    * last_update
    * content
* (delete)DeleteDetail
  * param
    * page name
    * seq
  * return: status

### parameter

#### msg
* a->c index[]
  * name
  * seq
  * description
* a->c info[]
  * name
  * bool_info[]
    * name
    * value
    * min
    * max
    * dflt
  * int_info[]
    * same as bool
  * doouble_info[]
    * same as bool
* c->a change[]
  * name
  * bool_change[]
    * name
    * value
  * int_info[]
    * same as bool
  * doouble_info[]
    * same as bool

#### API
* (get)GetHeaders
* (get)Getpage
* (put)SetPage

### telemetry

#### msg
* a->c index[]
  * name
  * interval
  * seq
  * description
* a->c content[]
  * name
  * seq
  * content

#### API
* (get)GetHeaders
  * param: none
  * return: index[]
    * name
    * status
    * description
* (get)GetDetail
  * param: page name
  * return: content
    * status
    * last_update
    * content

### setpoint

#### msg
* a->c index[]
  * name
  * timeout
  * seq
  * description
* c->a content
  * name
  * client_uuid
  * content

#### API
* (get)GetHeaders
  * param: none
  * return: index[]
    * name
    * status
    * description
* (put)PutDetail
  * param
    * name
    * content
  * return: status

### data
* u-list
* d-dmsg_data
* u-dmsg_ack
* u-umsg_data
* d-umsg_ack

### tile

#### msg
* a->c index[]
  * name
  * timeout
  * seq
  * description
  * keys[]
* a->c content
  * name
  * key
  * client_uuid
  * content

#### API
* (get)GetHeaders
  * param: none
  * return: index[]
    * name
    * status
    * description
    * keys[]
* (Get)GetDetail
  * param
    * name
    * key
    * content
  * return: status
