```plantuml
skinparam componentStyle rectangle
left to right direction

package "SRS0004_vehicle" {
  InterFace CanlinkBus as BUS

  component "[01]ShieldBoard" as SH
  component LattePanda as AP
  component YDLLidar as LIDAR
  component HeadCamera as Camera
  BUS -- SH
  SH -- AP
  AP -- LIDAR
  AP -- Camera
  AP -- WiFi

  component "[02]PowerBoard" as PW
  component Battery_x2 as BAT
  component WallPower as WALL
  BUS -- PW
  PW -- BAT
  PW -- WALL

  component "[03]WheelBoard" as WL
  component MotorEncoder_x3 as Mot
  component IMU as IMU
  BUS -- WL
  WL -- Mot
  WL -- IMU

  component "[04]GunBoard" as GN
  component AirGun as Gun
  component Magagine
  component Laser
  component Servo_x2 as Servo
  BUS -- GN
  GN -- Gun
  GN -- Magagine
  GN -- Laser
  GN -- Servo

  component "[05]UiBoard" as UI
  component switch as UI_SW
  component LED as UI_LED
  component PropoReceiver as SBUS2
  component Mag as Mag
  BUS -- UI
  UI -- UI_SW
  UI -- UI_LED
  UI -- SBUS2
  UI -- Mag

  component "[06]LineSensorBoard" as LN
  component PhotoRefrector_x9 as Line
  BUS -- LN
  LN -- Line

  component "[07]SonorBoard" as SN
  component Sonor_x2 as Sonor
  BUS -- SN
  SN --Sonor

  component "[08~10]TargetBoard_x3" as TR
  component Color_LED_x3 as TR_LED
  component AccelSensor as Accel
  BUS -- TR
  TR -- TR_LED
  TR -- Accel

  component "[11]LightBoard" as LT
  component LED as LT_LED
  BUS -- LT
  LT --LT_LED

}


```