%% RDC 2 Instance 1, RP chain.
clc;
clear;
%
my_r = rigidBodyTree;
body1 = rigidBody('Link_1');
jnt1 = rigidBodyJoint('jnt1','revolute');
body1.Mass = 10; %kg
body1.Joint = jnt1;

body2 = rigidBody('Link_2');
jnt2 = rigidBodyJoint('jnt2','prismatic');
body2.Mass = 6; %kg
body2.Joint = jnt2;

body3 = rigidBody('Link_3');
jnt3 = rigidBodyJoint('jnt3','fixed');
body3.Mass = 0; %kg
body3.Joint = jnt3;

addBody(my_r,body1,'base')
addBody(my_r,body2,'Link_1')
addBody(my_r,body3,'Link_2')

config = homeConfiguration(my_r);
config(1).JointPosition = pi/9; % rad
config(2).JointPosition = 0.2; % m
config(1).JointVelocity = 0.08; % rad/s
config(2).JointVelocity = 0.03; % m/s
config(1).JointAcceleration = 0.1; % rad/s^2
config(2).JointAcceleration = 0.01; % m/s^2
robot.ROT{1} = eye(3,3);
robot.ROT{2} = axang2rotm([0 0 1 pi/2]) * axang2rotm([1 0 0 pi/2]);
robot.R = my_r;
robot.C = config;
robot.r = [0, 1, 0; 
           0, 0, 0;
           0, 0, 0];
robot.Inertia = cell(2,1);
robot.Inertia{1,1} = diag([0, 0, 0.4]);
robot.Inertia{1,2} = diag([0, 0, 0.3]);
robot.r_com{1,1} = [0.5;0;0];
robot.r_com{1,2} = [0;0;0];
robot.r_com{1,3} = [0;0;0];

tau{1,1} = NewtEuler(robot, zeros(3,1), zeros(3,1), [0, -9.81, 0]);
tau{1,2} = NewtEuler(robot, zeros(3,1), zeros(3,1), [0, 0, 0]);