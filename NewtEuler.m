function [tau] = NewtEuler(robot,F_ext,M_ext,G)

    for i = 1:robot.R.NumBodies

        if(robot.R.Bodies{1,i}.Joint.Type == 'revolute')
        
        else(robot.R.Bodies{1,i}.Joint.Type == 'prismatic') 
            
        end
    
    end

   
end