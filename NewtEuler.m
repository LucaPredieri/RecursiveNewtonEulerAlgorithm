function [tau] = NewtEuler(robot,F_ext,M_ext,G)

    for i = 1:robot.R.NumBodies

        %Computing R
        q(i)=robot.C(i).JointPosition

        Rot=robot.R.Bodies{1,i}.Joint.ChildToJointTransform(1:3,1:3)*axang2rotm([0 0 1 q(i)]);

        if(robot.R.Bodies{1,i}.Joint.Type == 'revolute')
        
        elseif(robot.R.Bodies{1,i}.Joint.Type == 'prismatic') 
            
        end
    
    end

   
end