%% RDC 2 Instance 1, RR chain.
clc;
clear;
%
my_r = rigidBodyTree;
body1 = rigidBody('Link_1');
jnt1 = rigidBodyJoint('jnt1','revolute');
body1.Mass = 22; %kg
body1.Joint = jnt1;

body2 = rigidBody('Link_2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body2.Mass = 19; %kg
body2.Joint = jnt2;

body3 = rigidBody('Link_3');
jnt3 = rigidBodyJoint('jnt3','fixed');
body3.Mass = 0; %kg
body3.Joint = jnt3;

addBody(my_r,body1,'base')
addBody(my_r,body2,'Link_1')
addBody(my_r,body3,'Link_2')

config = homeConfiguration(my_r);
config(1).JointPosition = pi/2;
config(2).JointPosition = pi/4;
config(1).JointVelocity = -0.8;
config(2).JointVelocity = 0.35;
config(1).JointAcceleration = -0.4;
config(2).JointAcceleration = 0.1;
robot.ROT{1} = eye(3,3);
robot.ROT{2} = eye(3,3);
robot.R = my_r;
robot.C = config;
robot.r = [0, 1, 0.8; 
           0, 0, 0;
           0, 0, 0];
robot.Inertia = cell(2,1);
robot.Inertia{1,1} = diag([0, 0, 0.4]);
robot.Inertia{1,2} = diag([0, 0, 0.3]);
robot.r_com{1,1} = [0.5;0;0];
robot.r_com{1,2} = [0.4;0;0];
robot.r_com{1,3} = [0;0;0];

tau{1,1} = NewtEuler(robot, zeros(3,1), zeros(3,1), [0, -9.81, 0]);
tau{1,2} = NewtEuler(robot, zeros(3,1), zeros(3,1), [0, 0, 0]);