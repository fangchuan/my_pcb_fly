7.6:AHRS移植成功，但是yaw还是不对。一开始初始yaw很大，但是转动变化都对。过了大概10s后yaw开始不正常。

7.7：原来是磁力计的结果init_mx,init_my,init_mz有误！草，居然能成错，明天还得在调一调kp,ki

7.8：是磁力计的问题，要校准磁力计！详情见三个MXgain,MYgain,MZgain
     roll、pitch做变化时还是会影响到yaw，不过不是很大，控制在10°范围内

7.9: 完成整个程序 的移植，但还没有用上路由器。
     终于能读出遥控器的油门信号了，因为IIC用的是P0口，我操作方向寄存器PM0不合理，
     导致Timer_Channel5的输入端口P0.5被设置为输出了！
7.10：上位机可以对板子进行各种设定，但是还没有输出到上位机

7.12: 磁力计校准系数与副本不同
      基本逻辑测试通过，开始调PID
      路由器不稳定，建立不起来WIFI，前20S输出乱七八糟信息导致后面接收信息出乱子,未解决这个Bug
7.13：
      调roll时发现回传的的数据有大错误，经过IMU验证，排除电机的电磁干扰。
      确定为震动干扰！5hz滤波效果不明显
7.14:
     减小ahrs中的kp,2.5或2.0，震荡明显减小
     mpu滤波系数改为42hz时效果比较好
     貌似中断打印数据会对姿态融合造成影响，但是原来串口输出姿态角时没发现有这个问题
     又貌似halfT就应该定为常数0.001
     第二天测试，失败。。。。。。
7.15：
     DMP已经ok了，yaw跟原来一样会漂个10度左右不过不影响
     早上：roll: P--4 P--0.13 I--0.006 D--0.012 IMAX--100
     晚上：pitch:P--2 P--0.2  I--0.006 D--0.004 IMAX--50有-7度静差！改变IMAX没什么变化
     roll\pitch混控效果很好：#define   STABILIZE_ROLL_P       2.0

                     #define   STABILIZE_PITCH_P      2.0

                     #define   STABILIZE_YAW_P        2.0
 
                     #define   RATE_ROLL_P            0.27
                     #define   RATE_ROLL_I            0.006
                     #define   RATE_ROLL_D            0.004
                     #define   RATE_ROLL_IMAX         100.0

                     #define   RATE_PITCH_P           0.2
                     #define   RATE_PITCH_I           0.006
                     #define   RATE_PITCH_D           0.004
                     #define   RATE_PITCH_IMAX        100.0

7.16:
     Z轴参数如下：#define   RATE_YAW_P             0.35
                  #define   RATE_YAW_I             0.006
                  #define   RATE_YAW_D             0.012
                  #define   RATE_YAW_IMAX          50.0
    上位机设置偏航角，响应有点慢,试飞时会偏45°左右
    roll\pitch参数有明显抽搐感，改为#define   RATE_ROLL_P            0.2
                                    #define   RATE_ROLL_D            0.001     

                                    #define   RATE_PITCH_P           0.2
                                    #define   RATE_PITCH_D           0.001
    晚上：
          #define   RATE_ROLL_P            0.20
          #define   RATE_ROLL_I            0.006
          #define   RATE_ROLL_D            0.001
          #define   RATE_ROLL_IMAX         100.0

          #define   RATE_PITCH_P           0.20
          #define   RATE_PITCH_I           0.006
          #define   RATE_PITCH_D           0.001
          #define   RATE_PITCH_IMAX        100.0

          #define   RATE_YAW_P             1.0
          #define   RATE_YAW_I             0//.006
          #define   RATE_YAW_D             0//.002
          #define   RATE_YAW_IMAX          50.0
     成功试飞！但是pitch轴有静差，一直往负俯仰方向飞。我把target_pitch = 350;还是向负方向飞
               X轴左右漂
7.18:
     修改Y轴外环参数，静差有明显改善：#define   STABILIZE_ROLL_P       2.0
                                      #define   STABILIZE_ROLL_I       0.5
                                      #define   STABILIZE_ROLL_IMAX    500.0

                                      #define   STABILIZE_PITCH_P      2.0
                                      #define   STABILIZE_PITCH_I      0.5
                                      #define   STABILIZE_PITCH_IMAX   1000.0
     X轴模糊不定
     高度测量ok，移植高度PIDok
     修改角速度内环get_i函数，有效修正静差，而且系统稳定性增加
7.19：
     高度控制放弃内外环，采用单层。
7.21：
     高度控制周期改为20ms，基础油门为1450.
     上位机设定高度OK
7.22：
     重新调节yaw，原来参数太大了，会导致机身剧烈抖动。现在给yaw_pid_out限制在100内
     高度修改PID部分做了点变化。
     高度控制效果不明显，感觉D给的太大了，而且基础油门跟电量有关。明天把p\d参数的输出打印出来看看。
     d的控制有一个低通滤波的操作，可以尝试。
7.23：
     超声波测的数据有错，已更换。
     get_d()中有一个低通滤波参数我一直没加进去！也就是说d原来一直不起作用.修改过之后明显改善高度控制
     基础油门定于1520，与电池电压有关，已经通过单片机AD测得第一块电池电压，但是AD转换太耗时间。
     高度P:0.8
         D:0.4
7.24：
     换了个电调，感觉更好点。
     基础油门给大了，一直往上飞满电状态下1450basis就能起飞，而弱电状态下1520才能起飞
7.25：
     高度参数
             #define   STABILIZE_HIGH_P        1.0
             #define   STABILIZE_HIGH_I        0.006
             #define   STABILIZE_HIGH_D        0.45 
             #define   STABILIZE_HIGH_IMAX     20
   12.4V以上基础油门410，11.9V---12.4V之间基础油门430,11.7V以下基础油门450.
   定高60cm稳在30cm,定高70cm稳在50cm。设置高度响应平缓
7.26：
     又测试了高度PID，效果一般。把AD检测电池电压加进程序里了
7.28:
     光流用起来了，高于30cm时才开始输出位移，将摄像头视角改为45°。输出位移正常
     上位机增加修改位置PID功能、设置位移功能，下位机也对应功能。
7.29：
     #define   STABILIZE_POS_X_P         0.25
     #define   STABILIZE_POS_X_I         0
     #define   STABILIZE_POS_X_D         0.39或0.37
     #define   STABILIZE_POS_X_IMAX      2

     #define   STABILIZE_POS_Y_P         0.25
     #define   STABILIZE_POS_Y_I         0
     #define   STABILIZE_POS_Y_D         0.39或0.37
     #define   STABILIZE_POS_Y_IMAX      2
     roll、pitch在±5°抖动，不过视频中看不出来。位置在（0,0）原点抖动
     横滚的目标角设为初始角+0.5，但仍然往X轴负方向飞。
7.30：
     PCB上了新机架，但是飞不稳，怀疑是板子布局影响了9150内部工作。
7.31：
     给飞行器写自主飞行任务
     PCB板仍然抖动。
8.1：
     重新用电路板，不会出现发抖现象，确定为PCB板影响了9150数据。
     定高的基础油门阈值重新测了遍：12.6~12.2    用360
                                   12.2~11.9    用380
                                   低于11.9     用400
     光流初始化要加延时，否则上电初始化失败
     
8.3：
    第二版两个地线没连，其他正常。
    加上AD定高，效果还好。
    黑白格子上高度读取有问题，总是向y轴负方向飞。
8.4:
    超声波坏了，换了之后好了。但是加上AD功能后出现掉高现象，明天把AD值跟基础油门
    处理成线性关系试试。
    把y轴内环i至零。不再明显的往Y轴负方向飞。
8.6：
    330机架重新定高  #define   STABILIZE_HIGH_P        1.5
                     #define   STABILIZE_HIGH_I        0.02
                     #define   STABILIZE_HIGH_D        0.65
                     #define   STABILIZE_HIGH_IMAX     20
    超声波用手挡住会出现5m多的数据，这不应该啊！
    小机架肥的稍微高点还是会抖！要脱杆子给他飞微调
8.7：
    定高时X轴小抖，电池电压还是问题！
    定点把P减小了点，还是会画小圆圈。
    定点就这样吧#define   STABILIZE_POS_X_P         0.18
                #define   STABILIZE_POS_X_I         0
                #define   STABILIZE_POS_X_D         0.28
                #define   STABILIZE_POS_X_IMAX      1

                #define   STABILIZE_POS_Y_P         0.18
                #define   STABILIZE_POS_Y_I         0
                #define   STABILIZE_POS_Y_D         0.30
                #define   STABILIZE_POS_Y_IMAX      1
    X轴加上i后确实有效果，但是还是会漂到负方向。
    有一个电池放电很快。
8.8:
    改成按键选择模式。key2---定高   key3---定点
8.10：
    新板子定点定不住，用回8.8程序和板子。电压低的时候定点更稳，加大角度外环P无明显效果
8.11：
    新板子定点测试20遍，有时候还是会荡，发现Roll的目标角响应不好，
    存在荡的厉害而且有静差的现象，卸下来重新调roll。
8.14：
    循迹PID还有点抖，任务规划已经做好。
8.20：
     修改走矩形任务，修改捡铁块任务的降落延时，修改了yaw内环的P：0.2
8.22:
     外环参数全部改为4.0,不是很糟，但是yaw给90°仍然会翻