# 项目文件说明

## 文件树

```shell
.
├── code                        #代码
│   ├── workspace                   #工作空间
│   │   └── src                         #资源文件夹
│   │       ├── jubot                       #jubot相关包
│   │       │   ├── jubot_driver                #jubot车底盘控制包
│   │       │   │   └── ...
│   │       │   └── jubot_driver_interfaces     #封装接口，未用到
│   │       ├── mypkg                       #自定义功能包
│   │       │   ├── acquisition                 #摄像头采集
│   │       │   ├── interfaces                  #未用到接口
│   │       │   ├── line_follow                 #循线功能包
│   │       │   ├── navigation                  #导航，目前无法使用，待完善
│   │       │   ├── obstacle_avoidance          #避障功能包
│   │       │   ├── pubsub                      #订阅者发布者例子
│   │       │   └── service                     #服务端客户端例子
│   │       └── sllidar_ros2                #雷达相关包
├── data                        #报告
│   ├── made.md                     #项目报告原始文件markdown格式
│   └── made.pdf                    #报告pdf文件格式
└── README.md                   #说明文件

# 总计：73文件夹,196文件
去除修改底盘控制与雷达，仅统计mypkg相关包
-------------------------------------------------------------------------------
Language                     files          blank        comment           code
-------------------------------------------------------------------------------
C/C++ Header                     7            112            162            523
Python                          11             50             30            238
JSON                             6              0              0            186
XML                              8             36             27            167
CMake                            7             87            304            141
C++                              8             14             46            109
Lua                              2             12             13             75
YAML                             2              0              0              6
Gencat NLS                       2              0              0              3
-------------------------------------------------------------------------------
SUM:                            53            311            582           1448
-------------------------------------------------------------------------------

```
