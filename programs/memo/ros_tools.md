# ROS Tools

```plantuml
skinparam monochrome true

usecase Web
Web -> process
usecase Qt
usecase Rviz
usecase rqt

frame process{
    frame アクチュエーター{
        
    }
    frame センサー{

    }
}



```


```plantuml
robust "A相" as A
robust "B相" as B

@0
A is 0
B is 0

@10
A is 0
B is 1

@20
A is 1
B is 1

@30
A is 1
B is 0

@40
A is 0
B is 0

@50
A is 0
B is 1

@60
A is 1
B is 1

@70
A is 1
B is 0

@80
A is 0
B is 0
@enduml
```