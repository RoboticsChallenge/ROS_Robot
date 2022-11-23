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

% Creating seriallink object
RobotArm = SerialLink(L,'name', 'RobotArm');

% Need to set the joint limits
RobotArm.qlim = [[-2.1817 2.1817];[-3.4907 1.1345];[-0.6109 4.1015];[-pi pi];[-1.3090 2.6180];[-pi pi]]; 


% ROS
%%
% Starting ROS
rosinit

% Initializing global variables
global G;
global T;
T = rosmessage('geometry_msgs/Twist');
G(1) = 0;
G(2) = 0;
% Creating subscribers
sub_Pose = rossubscriber("/ros_robot/ManipulatorPose/Command",'geometry_msgs/Twist',@Twist_callback);
sub_gripper = rossubscriber("/ros_robot/Gripper/Command",'std_msgs/Float64',@Gripper_callback);

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
q_from = [0,0,0,0,0,0];
q_to = [0,0,0,0,0,0];
Last_T = rosmessage('geometry_msgs/Twist');
Last_T.Linear.X = 0;
Last_T.Linear.Y = 0;
Last_T.Linear.Z = 0;
Last_T.Angular.X = 0;
Last_T.Angular.Y = 0;
Last_T.Angular.Z = 0;


% Publishing rate 20Hz
rate = robotics.Rate(20);

% Calculate and publish data
% Will exit and shutdown ros if we publish Linear.X > 1000
while T.Linear.X < 1000

    % Checks if new message has been recived with wanted pose for robotarm
    % if new message has been recived, matlab will send nessecary joint
    % angles following a trajectory calaulated in matlab
    if(T ~= Last_T)        
        % Create transformation matrix for wanted position and orientation
        Transform = transl(T.Linear.X,T.Linear.Y,T.Linear.Z)*rpy2tr(T.Angular.X,T.Angular.Y,T.Angular.Z,'deg');
        
        % inverse kinematics to find nessecary joint angles for wanted
        % pose
        % We have had some issues with error messages when running this
        % command with some poses. The erros is stating that it is an
        % iligal pose even though it is possible (tested with teach
        % function)
        q_to = RobotArm.ikine(Transform);

        % Making a trajectory to wanted pose
        % Trajectory from "q_from" --> "q_to" with 200 points
        Trajectory = jtraj(q_from, q_to, 200); 

        % publishing joint angles to arm 
        for i=1: length(Trajectory)

            msg_q1.Data = Trajectory(i,1);
            msg_q2.Data = Trajectory(i,2);
            msg_q3.Data = -Trajectory(i,3);
            msg_q4.Data = Trajectory(i,4);
            msg_q5.Data = -Trajectory(i,5);
            msg_q6.Data = Trajectory(i,6);

            % Publish joint angles
            send(pub_q1,msg_q1);
            send(pub_q2,msg_q2);
            send(pub_q3,msg_q3);
            send(pub_q4,msg_q4); 
            send(pub_q5,msg_q5); 
            send(pub_q6,msg_q6); 

            waitfor(rate);
        end 
    end
    
    q_from = q_to;
    Last_T = T;

    msg_JawL.Data = G(1);
    msg_JawR.Data = G(2);

    % publish gripper position
    send(pub_JawL,msg_JawL);
    send(pub_JawR,msg_JawR);
    
    waitfor(rate);
end

% Shutdown ROS
rosshutdown


% Callback functions
% Pose command callback
% Recives (x,y,z,r,p,y) as a Twist message
function Twist_callback(src,msg)
    global T
    T = msg; 
end
% Gripper command callback
% Recives +-100% gripper command
% -100% = fully closed
% +100% = fully opened
% Convert to +-5cm command for each prismatic joint on the robotarm
function Gripper_callback(src,msg)
    global G
    G(1) = msg/20;
    G(2) = -msg/20;
end
