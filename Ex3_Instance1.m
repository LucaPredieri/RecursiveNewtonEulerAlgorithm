%% RDC 2 Instance 3, RRR chain.
clc;
clear;
%
my_r = rigidBodyTree;
body1 = rigidBody('Link_1');
jnt1 = rigidBodyJoint('jnt1','revolute');
body1.Mass = 20; %kg
body1.Joint = jnt1;

body2 = rigidBody('Link_2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body2.Mass = 20; %kg
body2.Joint = jnt2;

body3 = rigidBody('Link_3');
jnt3 = rigidBodyJoint('jnt3','revolute');
body3.Joint = jnt3;
body3.Mass = 6; %kg

body4 = rigidBody('End_Effector');
jnt4 = rigidBodyJoint('jnt4','fixed');
body4.Mass = 0; %kg
body4.Joint = jnt4;

addBody(my_r,body1,'base')
addBody(my_r,body2,'Link_1')
addBody(my_r,body3,'Link_2')
addBody(my_r,body4,'Link_3')

config = homeConfiguration(my_r);
config(1).JointPosition = pi/9;
config(2).JointPosition = 2*pi/9;
config(3).JointPosition = pi/18;
config(1).JointVelocity = 0.2;
config(2).JointVelocity = 0.15;
config(3).JointVelocity = -0.2;
config(1).JointAcceleration = 0.1;
config(2).JointAcceleration = 0.085;
config(3).JointAcceleration = 0;
robot.R = my_r;
robot.C = config;
robot.ROT{1} = eye(3,3);
robot.ROT{2} = axang2rotm([1 0 0 pi/2]);
robot.ROT{3} = eye(3,3);
robot.R.Gravity = [0 ; 0 ; -9.81]; %Gravity on z axis!!
robot.r = [0, 1, 0.8, 0.35; 
           0, 0, 0,   0;
           0, 0, 0,   0];
robot.Inertia = cell(1,3);
robot.Inertia{1,1} = diag([0.2,0.2,0.8]);
robot.Inertia{1,2} = diag([0.2,0.2,0.8]);
robot.Inertia{1,3} = diag([0.08,0.08,0.1]);
robot.r_com{1,1} = [0.5;0;0];
robot.r_com{1,2} = [0.4;0;0];
robot.r_com{1,3} = [0.35/2;0;0];

tau{1,1} = NewtEuler(robot, zeros(3,1), zeros(3,1), [0, 0, -9.81]);
tau{1,2} = NewtEuler(robot, zeros(3,1), zeros(3,1), [0, 0, 0]);