function [tau] = NewtEuler(robot,F_ext,M_ext,gravity)

    % Warnings are because variables are not initialized.

    k = [0;0;1];
    robot.R.Gravity = gravity;
    Rotot = zeros(3,3,3);
    r_com{1,1} = robot.r_com{1,1};
    r_com{1,2} = robot.r_com{1,2};
    r_com{1,3} = robot.r_com{1,3};
    
    % FORWARD RECURSION, computing rotation matrices, distances, 
    % velocities, accelerations of the joints of the robot with respect
    % to frame 0.

    for i = 1:robot.R.NumBodies-1
        j_type = robot.R.Bodies{1,i}.Joint.Type;
        q{1,i}=robot.C(i).JointPosition;
        
        % If the joint is revolute we have to consider the rotation of the 
        % link. If the joint is prismatic we have the base rotation of the 
        % joint (R_i for professor).
        
        if(strcmp(j_type,'revolute'))
             Rot{i} = robot.ROT{i} * axang2rotm([0 0 1 q{1,i}]);
        elseif(strcmp(j_type,'prismatic'))
             Rot{i} = robot.ROT{i};
        end
        if i == 1
            Rotot(:,:,i) = Rot{i};
        else 
            Rotot(:,:,i) =  Rotot(:,:,i-1)*Rot{i};
        end 
        
        % If the joint is revolute we will have some equations if the joint
        % is prismatic we have some other.

        if(strcmp(j_type,'revolute'))
               
             % For the first joint we'll have the vectors r,W and V equal
             % to zero because we have no distance and between the base
             % frame and the first frame and all the prior velocities are
             % equal to zero.

             if i == 1
                r{1,i} = zeros(3,1); 
                W{1,i} = zeros(3,1) + Rotot(:,:,i)*k*robot.C(i).JointVelocity;
                V{1,i} = zeros(3,1);
                dW{1,i} = Rotot(:,:,i)*k * robot.C(i).JointAcceleration;
                dV{1,i} = zeros(3,1);
             else
                r{1,i} = Rotot(:,:,i-1) * robot.r(:,i);
                W{1,i} = W{1,i-1} + Rotot(:,:,i)*k*robot.C(i).JointVelocity;
                V{1,i} = V{1,i-1} + cross(W{1,i-1},r{1,i});
                dW{1,i} = (dW{1,i-1} + cross(W{1,i-1},Rotot(:,:,i)*k)*robot.C(i).JointVelocity + Rotot(:,:,i)* k * robot.C(i).JointAcceleration);
                dV{1,i} = dV{1,i-1} + cross(dW{1,i-1},r{1,i}) + cross(W{1,i-1}, cross(W{1,i-1}, r{1,i}));
             end 
        
        elseif(strcmp(j_type,'prismatic'))

            % For the first joint we'll have the vectors r,W and V equal
            % to zero because we have no distance and between the base
            % frame and the first frame and all the prior velocities are
            % equal to zero.

            if i == 1
                r{1,i} = Rotot(:,:,i)*k*q{1,i}; 
                W{1,i} = zeros(3,1);
                V{1,i} = zeros(3,1) + Rotot(:,:,i)*k*robot.C(i).JointVelocity;
                dW{1,i} = zeros(3,1);
                dV{1,i} = Rotot(:,:,i)*k*robot.C(i).JointAcceleration;
            else     
                r{1,i} = Rotot(:,:,i-1) * robot.r(:,i) + Rotot(:,:,i)*k*q{1,i}; 
                W{1,i} = W{1,i-1};
                V{1,i} = V{1,i-1} + cross(W{1,i-1},r{1,i}) + Rotot(:,:,i)*k*robot.C(i).JointVelocity;
                dW{1,i} = dW{1,i-1};
                dV{1,i} = (dV{1,i-1} +cross(dW{1,i-1},r{1,i}) + cross(W{1,i-1},cross(W{1,i-1},r{1,i})) + cross(2*W{1,i-1},Rotot(:,:,i)*k*robot.C(i).JointVelocity) + Rotot(:,:,i)*k*robot.C(i).JointAcceleration);
             end 
        end

        % Computing the velocities of the center of masses with respect to
        % base frame.

        dVc{1,i} = dV{1,i} + cross(dW{1,i},Rotot(:,:,i)*r_com{1,i})+ cross(W{1,i},cross(W{1,i},Rotot(:,:,i)*r_com{1,i}));
    end
    
    % BACKWARDS RECURSION, computing forces, deltas and momentus 
    % of the joints of the robot with respect to frame 0.

    for i = robot.R.NumBodies-1:-1:1
        j_type = robot.R.Bodies{1,i}.Joint.Type;
        
        % Computing the final taus of the chain, considering the momentum
        % for the revolute joints and the forces for the prismatic joints.
        % As you can see, the Inertia matrix was premultiplied by Rotot(i) 
        % and post multiplied by Rotot(i)' which is the rotational matrix 
        % from frame i to base frame.

        if i == robot.R.NumBodies-1
            F{1,i} = robot.R.Bodies{1,i}.Mass*dVc{1,i}-robot.R.Bodies{1,i}.Mass*gravity' - F_ext ;
            delta{1,i} = Rotot(:,:,i)*robot.Inertia{1,i}* Rotot(:,:,i)'*dW{1,i}+cross(W{1,i},Rotot(:,:,i)*robot.Inertia{1,i}*Rotot(:,:,i)'*W{1,i});
            M{1,i} = delta{1,i} - M_ext - cross(-Rotot(:,:,i)*r_com{1,i},F{1,i});
        else
            F{1,i} = robot.R.Bodies{1,i}.Mass*dVc{1,i}-robot.R.Bodies{1,i}.Mass*gravity' -F_ext+F{1,i+1};
            delta{1,i} = Rotot(:,:,i)*robot.Inertia{1,i}*Rotot(:,:,i)'*dW{1,i}+cross(W{1,i},robot.Inertia{1,i}*W{1,i});
            M{1,i} = M{1,i+1} - M_ext - cross(-Rotot(:,:,i)*r_com{1,i},F{1,i}) + cross(r{1,i+1}-Rotot(:,:,i)*r_com{1,i},F{1,i+1})+ delta{1,i};
        end

        % If the joint is a revolute, we get momentum, if it is a
        % prismatic, we get the force.

        if(strcmp(j_type,'revolute'))
            axis = Rotot(:,:,i)*k;
            tau{1,i} = dot(M{1,i},axis);
        elseif(strcmp(j_type,'prismatic'))
            axis = Rotot(:,:,i)*k;
            tau{1,i} = dot(F{1,i},axis);
        end
    end
end