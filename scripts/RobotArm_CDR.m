clear, close all;
clc;

% Link lengths
L1 = 0.162575;
L1_d = 0.112;
L1_a = 0.117839;
L2 = 0.28;
L3 = 0.186904;
L4 = 0.109554;
L5 = 0.08561; 
L6 = 0.116047;

% Link Offsets
L1_offset = pi;
L2_offset = 2.35619;
L3_offset = -0.7853734;
L5_offset = 0.68033;

% Creating Links with DH - parameters
L(1) = Link('d',L1_d,'a',L1_a,'alpha',pi/2,'offset',L1_offset);
L(2) = Link('d',0,'a',L2,'alpha',0,'offset',L2_offset);
L(3) = Link('d',0,'a',0,'alpha',pi/2,'offset',L3_offset);
L(4) = Link('d',L3+L4,'a',0,'alpha',pi/2,'offset',0);
L(5) = Link('d',0,'a',0,'alpha',pi/2,'offset',L5_offset);
L(6) = Link('d',-(L5+L6),'a',0,'alpha',0,'offset',0);

RobotArm = SerialLink(L,'name', 'RobotArm');

% Need to set the joint limits
RobotArm.qlim = [[-2.1817 2.1817];[-3.4907 1.1345];[-0.6109 4.1015];[-pi pi];[-1.3090 2.6180];[-pi pi]]; 

% ROS
%%
% Starting ROS
rosinit

% Initializing publishers
[pub_q1,msg_q1] = rospublisher('/ros_robot/L1_position_controller/command','std_msgs/Float64');
[pub_q2,msg_q2] = rospublisher('/ros_robot/L2_position_controller/command','std_msgs/Float64');
[pub_q3,msg_q3] = rospublisher('/ros_robot/L3_position_controller/command','std_msgs/Float64');
[pub_q4,msg_q4] = rospublisher('/ros_robot/L4_position_controller/command','std_msgs/Float64');
[pub_q5,msg_q5] = rospublisher('/ros_robot/L5_position_controller/command','std_msgs/Float64');
[pub_q6,msg_q6] = rospublisher('/ros_robot/L6_position_controller/command','std_msgs/Float64');
[pub_JawL,msg_JawL] = rospublisher('/ros_robot/JawL_position_controller/command','std_msgs/Float64');
[pub_JawR,msg_JawR] = rospublisher('/ros_robot/JawR_position_controller/command','std_msgs/Float64');

% Initialize variables
q_temp = [0,0,0,0,0,0];

% Publishing rate 10Hz
rate = robotics.Rate(10);


for i = 1:length()
end



% Calculate and publish data
% Will exit and shutdown ros if we publish Linear.X > 1000
while rate.TotalElapsedTime < 10 %T.Linear.X < 1000
      
   
        msg_q1.Data = q_temp(1);
        msg_q2.Data = q_temp(2);
        msg_q3.Data = q_temp(3);
        msg_q4.Data = q_temp(4);
        msg_q5.Data = q_temp(5);
        msg_q6.Data = q_temp(6);
        msg_JawL.Data = 0;
        msg_JawR.Data = 0;

        % Publish
        send(pub_q1,msg_q1);
        send(pub_q2,msg_q2);
        send(pub_q3,msg_q3);
        send(pub_q4,msg_q4); 
        send(pub_q5,msg_q5); 
        send(pub_q6,msg_q6); 
        send(pub_JawL,msg_JawL);
        send(pub_JawR,msg_JawR);

%     if(T ~= Last_T)
%         % Kode til SexyLars
% 
%         Transform = transl(T.Linear.X,T.Linear.Y,T.Linear.Z)*rpy2tr(T.Angular.X,T.Angular.Y,T.Angular.Z,'deg');
% 
%         q_temp = RobotArm.ikine(Transform);
%         msg_q1.Data = q_temp(1);
%         msg_q2.Data = q_temp(2);
%         msg_q3.Data = q_temp(3);
%         msg_q4.Data = q_temp(4);
%         msg_q5.Data = q_temp(5);
%         msg_q6.Data = q_temp(6);
% 
%         % Publish
%         send(pub_q1,msg_q1)
%         send(pub_q2,msg_q2)
%         send(pub_q3,msg_q3)
%         send(pub_q4,msg_q4) 
%         send(pub_q5,msg_q5) 
%         send(pub_q6,msg_q6) 
% 
%     end
% 
%     Last_T = T;
    
    waitfor(rate);
end

% Shutdown ROS
rosshutdown
