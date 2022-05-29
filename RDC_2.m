
%% RDC 2 

%%
dhparams = [0   	0  0   	0;
        1	0   0     0 
        0.8   0  0     0 ];

my_r = rigidBodyTree;
body1 = rigidBody('Base');
jnt1 = rigidBodyJoint('jnt1','fixed');

setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;
body1.Mass = 22; %kg
basename=my_r.BaseName;
addBody(my_r,body1,basename)

body2 = rigidBody('Link_1');
jnt2 = rigidBodyJoint('jnt2','revolute');
setFixedTransform(jnt2,dhparams(2,:),'dh');
body2.Joint = jnt2;
body2.Mass = 19; %kg

body3 = rigidBody('Link_2');
jnt3 = rigidBodyJoint('jnt3','revolute');
setFixedTransform(jnt3,dhparams(3,:),'dh');
body3.Joint = jnt3;

addBody(my_r,body2,'Base')
addBody(my_r,body3,'Link_1')

collisionObj = collisionCylinder(0.05,0.3);
end_eff = collisionSphere(0.1);

for i = 1:my_r.NumBodies
    
    if(i==3)
        addCollision(my_r.Bodies{i},end_eff);
    else
        addCollision(my_r.Bodies{i},collisionObj);
    end
   
end
%%
config = homeConfiguration(my_r);
config(1).JointPosition = pi/9;
config(2).JointPosition = 2*pi/9;
config(1).JointVelocity = 0.2;
config(2).JointVelocity = 0.15;
config(1).JointAcceleration = 0.1;
config(2).JointAcceleration = 0.085;


robot.R = my_r;
robot.C = config;

my_r.Gravity = [0 0 -9.81];

%my_r.DataFormat = 'row';

q = randomConfiguration(my_r);
%tau = inverseDynamics(my_r,config)
showdetails(my_r)
figure()
show(my_r,config,'Collisions','on','Visuals','off');


