# logit architecture

# 分類

logicは以下の4つにわかれる。
* StateMachine
  * 定義
    * Parameter
    * Data(条件)
    * Result
    * Event
    * State
  * メソッド
    * *Parameter SetParam()
    * *Data SetData()
    * Result request(Event)
    * State GetState()

* Detector
  * 定義
    * Parameter
    * Result
    * Input
  * メソッド
    * *Parameter SetParam()
    * Result Detect(Input)

* StatelessFilter
  * 定義
    * Parameter
    * Data(状況マップ)
    * Output
    * Input
    * Result
  * メソッド
    * *Parameter SetParam()
    * *Data SetData()
    * Result Run(Stamp, Input, Output&)

* HistoricalFilter
  * 定義
    * Parameter
    * Data(状況マップ)
    * Output
    * Input
    * Result
  * メソッド
    * *Parameter SetParam()
    * *Data SetData()
    * Result Run(Stamp, Input, Output&)

## 表

### 定義
||StateMachine|Detector|StatelessFilter|HistoricalFilter|
|:--|:--|:--|:--|:--|
|Parameter|o|o|o|o|
|Result|o|o|o|o|
|Data|o(condition)|x|o(map,odom)|o(map,odom)|
|Event|o|x|x|x|
|State|o|x|x|x|
|Input|x|o|o|o|
|Output|x|x|o|o|

### メソッド
||StateMachine|Detector|StatelessFilter|HistoricalFilter|
|:--|:--|:--|:--|:--|
|*Parameter SetParam()|o|o|o|o|
|*Data SetData()|o|x|o|o|
|Result request(Event)|o|x|x|x|
|State GetState()|o|x|x|x|
|Result Detect(Input)|x|o|x|x|
|Result Run(Stamp, Input, Output&)|x|x|o|o|

# できるべきこと
* Paramのコンソールから読み書き
* 実行されたlogicの履歴
    * common: 時刻と実行時間
    * StateMachine: eventとresultとstate
    * Detector: Result
    * StatelessFilter: Result
    * HistoricalFilter: Result

# 例

* perception
    * HSVFilter
    * Tracking
    * rect to ray
    * obstaccle detector
    * alvar detector
* recognition
    * obstacle map
    * SLAM
    * topological map
    * pose ekf
    * 
* Task Planning
    * sequencer
* Local Planning
    * topological
    * A*, RRT
    * ElasticBand
* control
    * DWA
    * Pose(PID)
    * MPC
    * Manual
