close all
clc
T = 20;                %Number of Iteration
l = 0.9;                % Step-size
eta = 0.99;             % Step-controlling factor        
d = 0.9;                % Antenna Length

R=0.02; c=2*pi*R; q=0.037; L=2*0.165;
 
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected')

    %Handle
    [returnCode,start]=vrep.simxGetObjectHandle(clientID,'Start',vrep.simx_opmode_blocking);
    [returnCode,goal]=vrep.simxGetObjectHandle(clientID,'Goal',vrep.simx_opmode_blocking);
    [returnCode,robot]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    [returnCode,left_motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_leftMotor',vrep.simx_opmode_blocking);
    [returnCode,right_motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_rightMotor',vrep.simx_opmode_blocking);
    [returnCode,sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking);
    
    % action
   
    
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,robot,start,vrep.simx_opmode_blocking);
        [returnCode,goal_position]=vrep.simxGetObjectPosition(clientID,goal,start,vrep.simx_opmode_blocking);
        
        
        position(:,3)=[];
        pose = position;
        goal_position(:,3)=[];
       
        D = [1,2];

        x = rand(D);            %Current Position of Robot
        x_goal = goal_position;    % Goal Position of Robot      
        x_init = x;             % Start Position of Robot
        
        f = @(x)func(x, x_goal);% Objectve Function Handle
        f_min = f(x);           % Initial value of Objective Function
 
        X = [];
        F = [];
        f_best = [];
        x_best = rand(D);
 
        for t = 1:T
 
            b = rand(D);
 
            x_r = x + d*b;
            x_l = x - d*b;
            
            p = d*(b*l).*(f(x_r) - f(x_l));
            
            x = x - p;
            
            fnc = f(x);
            
            d = eta*d + 0.01;
            l = eta*l;
            
            if fnc < f_min
                f_best = [f_best; fnc];
                x_best = [x_best; x];
                f_min = fnc;
            end
            
            X = [X; x];
            F = [F; fnc];
            
            
          
        end
        
       s = 0;
          for k=1:size(x_best(:,1))
              v = 1; % constant velocity
              % current position
              [returnCode,euler]=vrep.simxGetObjectOrientation(clientID,robot,start,vrep.simx_opmode_blocking);
              [returnCode,position]=vrep.simxGetObjectPosition(clientID,robot,start,vrep.simx_opmode_blocking);
              
               position(:,3) = [];
               theta= euler(1,3); 
                x_a = position(:,1); y_a = position(:,2); %current position
                
                x_b = x_best(k,1); y_b = x_best(k,2); %goal X,Y position
                % desired theta
              
                    theta_d = atan2((y_b-y_a), (x_b-x_a));
                
                % Error terms
                e = theta_d- theta;
                Kp = 50; 
                Ki = 0.3;
               e_i(k) = e+s;
               
                w = Kp*e+Ki*e_i; 
                    omega =w; 
                    
                    v_r = (2*v+omega*R)/(2*L);    %right wheel velocity
                    v_l = (2*v-omega*R)/(2*L);    %left wheel velocity
                    
                  % Joint velocities
                    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_motor,v_l,vrep.simx_opmode_blocking);
                    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_motor,v_r,vrep.simx_opmode_blocking);
                    
                    s = e+s; 
                   
                 
          end
          


                   
end
    
            
    vrep.simxFinish(-1);
    
    

vrep.delete();
    
          