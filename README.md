# Constant Curvature Model
 
## 1.使用前准备
* 在使用该函数之前请先下载，现代机器人学的算法程序，添加在Matlab的路径中 https://github.com/NxRLab/ModernRobotics

## 2.函数输入
本函数与[Design and Kinematic Modeling of Constant Curvature Continuum Robots: A Review](https://doi.org/10.1177/0278364910368147)文中使用[$`k`$;$`\phi`$;$`l`$]描述曲线参数略有不用，是使用[$`\theta`$;$`\phi`$;$`l`$]，$`\theta = kl/\pi\times180`$。

* theta_deg 弹性体的弯曲角$`\theta`$ (单位：°)
* phi_deg 弹性体的偏转角$`\phi`$ (单位：°)
* structure_param 连续体的硬件参数(结构体)
    * r_disk 每个盘片的半径 (单位：mm)
    * r_hole 穿绳孔与盘片圆心的距离 (单位：mm)
    * r_reel 缠绳辊的半径 (单位：mm)
    * lc 中心弹性体长度 $`l`$ (单位：mm)
    * n 中心弹性体分为n段
* origin 连续体的原点位置(4x4的齐次变换矩阵)
* ‘doplot’ 是否绘图，当该函数作为纯逆运动学函数时不绘图将提高程序运行速度
    * true 绘图
    * false 不绘图

## 3. 函数输出
以$`\theta = 0`$ 时作为三个电机的零位。
* theta1 1号腱驱动电机的转角
* theta2 2号腱驱动电机的转角
* theta3 3号腱驱动电机的转角
