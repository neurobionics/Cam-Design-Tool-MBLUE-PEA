function [I2Rloss_total,i2r,residualTorque,cubicTorque,IVenergyUse,IVpow,current] = I2Rcostfunc(X,motor_params,theta_alltasks,theta_d_alltasks,theta_dd_alltasks,stridetime_alltasks,exoTorque_alltasks,taskweights)
% Input coefficient vector, motor info, and theta, etc matrices where each
% column corresponds to a different task

%   Detailed explanation goes here
cx3d = X(1);
cx2d = X(2);
cx1d = X(3);
cx3p = X(4);
cx2p = X(5);
cx1p = X(6);
z = X(7);
tasks = length(theta_alltasks(1,:));

%preallocate for speed
% cubicTorque = zeros(length(theta),1);
% residualTorque = zeros(length(theta),1);
% current = zeros(length(theta),1);
% i2r = zeros(length(theta),1);

for j = 1:tasks
    theta = theta_alltasks(:,j);
    theta_d = theta_d_alltasks(:,j);
    theta_dd = theta_dd_alltasks(:,j);
    stridetime = stridetime_alltasks(:,j);
    exoTorque = exoTorque_alltasks(:,j);

    %% I2R Analysis
    
    for i = 1:length(theta)
        %use appropriate function for dorsi vs plantar torque/stiffness
        if theta(i)>z %dorsiflex
            cubicTorque(i,j) = cx3d*(theta(i)-z)^3 + cx2d*(theta(i)-z)^2 + cx1d*(theta(i)-z);
        else %plantarflex
            cubicTorque(i,j) = cx3p*(theta(i)-z)^3 + cx2p*(theta(i)-z)^2 + cx1p*(theta(i)-z);
        end
        %find what torque the actuator needs to contribute
        %for this theta step and quadratic cam stiffness
        residualTorque(i,j) = exoTorque(i) - cubicTorque(i,j);
        %from kt*i= J*thetadd + bm*thetad + T1
        current(i,j) = (motor_params.inertia*theta_dd(i) + motor_params.bm*theta_d(i) + residualTorque(i,j))/(motor_params.GR*motor_params.kt);
        i2r(i,j) = (current(i,j)^2)*motor_params.Rphase;
    
    end

    I2Rloss(1,j) = trapz(stridetime,i2r(:,j)); %integrate power loss over time -> E loss

    voltage = current(:,j)*motor_params.Rphase + motor_params.Le*ddt_open(current(:,j)) + motor_params.kt*theta_d;
    IVpow(:,j) = current(:,j).*voltage;
    IVenergyUse(1,j) = trapz(stridetime,IVpow(:,j));
end
weightedI2Rloss = taskweights.*I2Rloss;
I2Rloss_total = sum(weightedI2Rloss);

end