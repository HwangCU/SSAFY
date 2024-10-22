# Robot DK
![alt text](image.png)


## 유용한 명령어
```python
# 로봇팔 관련 명령어
RB1 = RDK.Item("RB1",itemtype=2) # 로봇 선언
RB1_tool = RDK.Item("RB1_tool",itemtype=4)

RB1_tool.AttachClosest("Box1",1000)
RB1_tool.DetachAll(parent=좌표계)

RB1.MoveL(조인트값)
RB1.MoveC(조인트값)
RB1.MoveJ(조인트값)

RB1.Pose()[0,x] # x = 0,1,2 x, y, z의 값
RB1.Joints()[0,x] # x = 0,1,2,3,4,5 조인트 값