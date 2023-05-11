%定义机器人参数
%-连续体原点位置
origin(1:3,4) = [10;10;0];    % x,y,z
origin(1:3,1:3) = rotz(30);
%-连续体结构参数
structure_param.r_disk = 17.5;  %每个盘片的半径
structure_param.r_hole = 15;    %穿绳孔与盘片圆心的距离
structure_param.r_reel = 4;     %缠绳辊的半径
structure_param.lc = 47*3;      %中心弹性体长度
structure_param.n = 3;          %中心弹性体分了几段

%定义输入参数(角度制)
%-弯曲角
theta = [30:10:100];
%-旋转角
phi = [0:10:360];

%绘图
fig = figure('Name','Demonstration');
view(3);
axis equal;
xlim([-100,100]);
ylim([-100,100]);
zlim([-0,200]);

%保存视频
v = VideoWriter('newfile.mp4','MPEG-4');
open(v)

for th = theta
    for ph = phi
        cla;
        [theta1,theta2,theta3] = Inverse_kinematic(th,ph,structure_param,origin,'dopolt',true);
        frame = getframe(gcf);
        writeVideo(v,frame);
        pause(0.05);
    end
end

close(v);