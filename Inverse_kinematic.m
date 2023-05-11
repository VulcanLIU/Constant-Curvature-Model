function [theta1,theta2,theta3] = Inverse_kinematic(theta_deg,phi_deg,...
    structure_param,origin,varargin)
%
% @author   Vulcanliu email:vulcanliu@outlook.com
% @brief    逆向运动学，输入连续体几何参数，输出三个舵机角
% @param1   theta_deg 弹性体的弯曲角
% @param2   phi_deg 弹性体的偏转角
% @param3   structure_param 连续体机械臂的硬件参数
% @param4   origin 机械臂原点 是 齐次变换矩阵
% @param5   origin 机械臂原点 是 齐次变换矩阵
% @return   theta1,theta2,theta3 三个舵机角，角度制
%

%找是否指定不绘图
doPlot = true; %默认为画图
doplot_index = find(strcmp(varargin,'doplot'));
if ~isempty(doplot_index)
    if cell2mat(varargin(doplot_index+1)) == false
        doPlot = false;
    end
end

circle_R = structure_param.r_disk;
circle_ang = 0:18:360;
circle_x = sin(deg2rad(circle_ang))*circle_R;
circle_y = cos(deg2rad(circle_ang))*circle_R;
circle_z = zeros(1,length(circle_x));
circle_position = [circle_x;circle_y;circle_z];

temp_flag = 0;
if theta_deg == 0||theta_deg == 360
    theta_deg = 0.01;
    temp_flag = 1;
end

%转存变量
theta_rad = theta_deg*pi/180;
phi_rad = phi_deg*pi/180;
d = structure_param.r_hole;
roller_r = structure_param.r_reel;
lc = structure_param.lc;%中心弹性体长度
n = structure_param.n;%一共分多少段

% 计算每个盘片的坐标系
% - 计算旋转轴
r = lc/theta_rad;
q = rotz(phi_deg)*[r;0;0];
s = rotz(phi_deg)*[0;-1;0];
S = ScrewToAxis(q,s,0);
% - 计算等效连杆
l_length = 2*r*sin(theta_rad/2);
dir = rotz(phi_deg)*roty(-90+theta_deg/2)*[1;0;0];
l = l_length*dir;

%
V1 = -S*theta_rad*0/n;
V2 = -S*theta_rad*1/n;
V3 = -S*theta_rad*2/n;
V4 = -S*theta_rad*3/n;

% - 计算拉绳长度
vector1 = rotz(0)*[0;d;0];
vector2 = rotz(0+120)*[0;d;0];
vector3 = rotz(0+240)*[0;d;0];
r1 = r- dot(vector1,q)/r;%中心体半径-投影
r2 = r- dot(vector2,q)/r;
r3 = r- dot(vector3,q)/r;
l1 = n*(r1*sin(theta_rad/n/2))*2;
l2 = n*(r2*sin(theta_rad/n/2))*2;
l3 = n*(r3*sin(theta_rad/n/2))*2;

% - 计算齐次变换矩阵
T(:,:,1) = MatrixExp6(VecTose3(V1));
T(:,:,2) = MatrixExp6(VecTose3(V2));
T(:,:,3) = MatrixExp6(VecTose3(V3));
T(:,:,4) = MatrixExp6(VecTose3(V4));

% - 计算每一个孔位位置
hole1_1 = vector1;
hole1_2 = vector2;
hole1_3 = vector3;
hole1_O = [0;0;0];

% 绘制
hold on;
grid on;

% % -绘制盘片坐标系
% for index1 = 1:n+1
%     %--第index1个盘片
%     %---计算盘片的原点位置
%     T_ = origin*T(:,:,index1);
%     for index2 = 1:3
%         %---第index2个坐标轴(xyz共三个)
%         %----(画法1)quiver3绘制向量画法
% %         quiver3(T_(1,4),T_(2,4),T_(3,4),...
% %             T_(1,index2),T_(2,index2),T_(3,index2),10,...
% %             'Color','r');
%         %----(画法2)plot3绘制直线画法
%         temp_n = 10; 
%         plot3([T_(1,4),T_(1,4)+temp_n*T_(1,index2)],...
%             [T_(2,4),T_(2,4)+temp_n*T_(2,index2)],...
%             [T_(3,4),T_(3,4)+temp_n*T_(3,index2)],...
%         'Color','r');
%         
%     end
% end

if doPlot
    % -绘制孔位和拉绳
    holes_before = zeros(4,4);

%     [R1,p1] = TransToRp(origin*T(:,:,1));
%     [R4,p4] = TransToRp(origin*T(:,:,4));
%     circle_1_position = R4*circle_position+p1;
%     plot3(circle_1_position(1,:),circle_1_position(2,:),circle_1_position(3,:),'Color','b');

    for index3 = 1:n+1
        %第index3个盘片
        hole_1 = origin*T(:,:,index3)*[hole1_1;1];
        hole_2 = origin*T(:,:,index3)*[hole1_2;1];
        hole_3 = origin*T(:,:,index3)*[hole1_3;1];
        hole_O = origin*T(:,:,index3)*[hole1_O;1];
        holes = [hole_1';hole_2';hole_3';hole_O'];

        [R,p] = TransToRp(origin*T(:,:,index3));
        circle_1_position = R*circle_position+p;
        plot3(circle_1_position(1,:),circle_1_position(2,:),circle_1_position(3,:),'Color','b');

        for index4 = 1:3
            %第index4个孔位
            plot3([hole_O(1),holes(index4,1)], ...
                [hole_O(2),holes(index4,2)], ...
                [hole_O(3),holes(index4,3)], ...
                'Color','black');
            if index3>1
                plot3([holes_before(index4,1),holes(index4,1)],...
                    [holes_before(index4,2),holes(index4,2)],...
                    [holes_before(index4,3),holes(index4,3)],...
                    'Color','g','LineStyle','-.');
            end
        end
        holes_before = holes;
    end
    % -绘制弹性体
    T_new = origin*T(:,:,1);
    %将弹性体分为arc_num段
    arc_num = 10;
    for index5 = 1:arc_num
        V_ = -S*theta_rad*index5/arc_num;
        T_before =  T_new;
        T_new = origin*MatrixExp6(VecTose3(V_));
        plot3([T_before(1,4),T_new(1,4)],[T_before(2,4),T_new(2,4)],[T_before(3,4),T_new(3,4)],...
            'Color','black','LineWidth',1.2);
    end
end

if temp_flag == 1
    l1 = lc;
    l2 = lc;
    l3 = lc;
end

% 输出
theta1 = (l1-lc)/roller_r*180/pi;
theta2 = (l2-lc)/roller_r*180/pi;
theta3 = (l3-lc)/roller_r*180/pi;
end

