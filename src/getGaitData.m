function [jointTorque,theta,theta_d,theta_dd,stridetime] = getGaitData(dataset,joint,task,subnum)
%% Converstion Factors
rev2rad = 2*pi;
min2sec = 1/60;
deg2rad = pi/180;
in2m = .0254;
lb2n = 4.4482216153;
%% Bring in angle and torque trajectories; format them 
% from Bovi
if isequal(dataset,'bovi')
    if isequal(joint,'ankle') %ankle
        if isequal(task,'normalwalk')  %normal walking
            %Natural walking speed, adult subject, Bovi data set
            theta = readmatrix('Bovi data.xls','Sheet','Joint Rotations','Range','AE710:AE810'); % mean-1sd AD, mean AE, mean+1sd AF
            jointTorque = readmatrix('Bovi data.xls','Sheet','Joint Moments','Range','AE407:AE507'); % mean-1sd, mean, mean+1sd
        elseif isequal(task,'ascent')  %ascent
            %ascent walking, adult subject, Bovi data set
            theta = readmatrix('Bovi data.xls','Sheet','Joint Rotations','Range','AZ710:AZ810'); % mean-1sd AD, mean AE, mean+1sd AF
            jointTorque = readmatrix('Bovi data.xls','Sheet','Joint Moments','Range','AZ407:AZ507'); % mean-1sd, mean, mean+1sd
        elseif isequal(task,'descent')  %descent
            %descent walking, adult subject, Bovi data set
            theta = readmatrix('Bovi data.xls','Sheet','Joint Rotations','Range','BC710:BC810'); % mean-1sd AD, mean AE, mean+1sd AF
            jointTorque = readmatrix('Bovi data.xls','Sheet','Joint Moments','Range','BC407:BC507'); % mean-1sd, mean, mean+1sd
        else
            disp('Invalid Bovi Ankle Task Selected')
        end
    elseif isequal(joint,'knee')
        if isequal(task,'normalwalk')  %normal walking
            %Natural walking speed, adult subject, Bovi data set
            theta = readmatrix('Bovi data.xls','Sheet','Joint Rotations','Range','AE609:AE709'); % mean-1sd AD, mean AE, mean+1sd AF
            jointTorque = readmatrix('Bovi data.xls','Sheet','Joint Moments','Range','AE205:AE305'); % mean-1sd, mean, mean+1sd
        elseif isequal(task,'ascent')  %ascent
            %Natural walking speed, adult subject, Bovi data set
            theta = readmatrix('Bovi data.xls','Sheet','Joint Rotations','Range','AZ609:AZ709'); % mean-1sd AD, mean AE, mean+1sd AF
            jointTorque = readmatrix('Bovi data.xls','Sheet','Joint Moments','Range','AZ205:AZ305'); % mean-1sd, mean, mean+1sd
        elseif isequal(task,'descent')  %descent
            %Natural walking speed, adult subject, Bovi data set
            theta = readmatrix('Bovi data.xls','Sheet','Joint Rotations','Range','BC609:BC709'); % mean-1sd AD, mean AE, mean+1sd AF
            jointTorque = readmatrix('Bovi data.xls','Sheet','Joint Moments','Range','BC205:BC305'); % mean-1sd, mean, mean+1sd
        else
            disp('Invalid Bovi Knee Task Selected')
        end   
    elseif isequal(joint,'hip')
        if isequal(task,'normalwalk')  %normal walking
            %Natural walking speed, adult subject, Bovi data set
            theta = readmatrix('Bovi data.xls','Sheet','Joint Rotations','Range','AE306:AE406'); % mean-1sd AD, mean AE, mean+1sd AF
            jointTorque = readmatrix('Bovi data.xls','Sheet','Joint Moments','Range','AE3:AE103'); % mean-1sd, mean, mean+1sd
        elseif isequal(task,'ascent')  %ascent walking
            %Natural walking speed, adult subject, Bovi data set
            theta = readmatrix('Bovi data.xls','Sheet','Joint Rotations','Range','AZ306:AZ406'); % mean-1sd AD, mean AE, mean+1sd AF
            jointTorque = readmatrix('Bovi data.xls','Sheet','Joint Moments','Range','AZ3:AZ103'); % mean-1sd, mean, mean+1sd
        elseif isequal(task,'descent')  %descent walking
            %Natural walking speed, adult subject, Bovi data set
            theta = readmatrix('Bovi data.xls','Sheet','Joint Rotations','Range','BC306:BC406'); % mean-1sd AD, mean AE, mean+1sd AF
            jointTorque = readmatrix('Bovi data.xls','Sheet','Joint Moments','Range','BC3:BC103'); % mean-1sd, mean, mean+1sd
        else
            disp('Invalid Bovi Hip Task Selected')
        end        
    else
        disp('Invalid Joint Selected')
    end

    %% Reformat Bovi
    
    %convert angle to radians
    theta = theta*deg2rad;
    %set time step
    dt = .01;
    strideduration = dt*(length(theta(:,1))-1);
    stridetime = 0:dt:strideduration; %[sec]
    
    %shift ankle angles to start trajectory at zero
    theta = (theta-theta(1));
    %calculate derivatives
    theta_d = ddt_open(theta,dt);
    theta_dd = ddt_open(theta_d,dt);
    %negate torque values to match proper sign convention
    jointTorque = -jointTorque;

%% Georgia Tech Dataset (2023)
elseif isequal(dataset,'GT2023')
    
    rootpath = append('C:\Users\neslerc\Downloads\Segmentation\',subnum,'\');
    
    if isequal(task,'normalwalk1x2')  %normal walking (1.2 m/s)
        filename = 'normal_walk_1_1-2_segmented.mat';
        fullpath = strcat(rootpath,filename);
        data = load(fullpath);
        if isequal(joint,'ankle')
            theta = data.angle.avg_l.sig_mu.ankle_angle_l;
            jointTorque = data.moment_filt.avg_l.sig_mu.ankle_angle_l_moment;
        elseif isequal(joint,'knee')
            theta = data.angle.avg_l.sig_mu.knee_angle_l;
            jointTorque = data.moment_filt.avg_l.sig_mu.knee_angle_l_moment;
        elseif isequal(joint,'hip')
            theta = data.angle.avg_l.sig_mu.hip_flexion_l;
            jointTorque = data.moment_filt.avg_l.sig_mu.hip_flexion_l_moment;
        else
            disp('Invalid GT2023 Joint Selected (Normal Walk 1.2 m/s)')
        end
        
    elseif isequal(task,'stairascent') %switched from R to L due to NaN in torque array (last two samples)
        filename = 'stairs_1_up_segmented.mat';
        fullpath = strcat(rootpath,filename);
        data = load(fullpath);
        if isequal(joint,'ankle')
            theta = data.angle.avg_l.sig_mu.ankle_angle_l;
            jointTorque = data.moment_filt.avg_l.sig_mu.ankle_angle_l_moment;
        elseif isequal(joint,'knee')
            theta = data.angle.avg_l.sig_mu.knee_angle_l;
            jointTorque = data.moment_filt.avg_l.sig_mu.knee_angle_l_moment;
        elseif isequal(joint,'hip')
            theta = data.angle.avg_l.sig_mu.hip_flexion_l;
            jointTorque = data.moment_filt.avg_l.sig_mu.hip_flexion_l_moment;
        else
            disp('Invalid GT2023 Joint Selected (Stair Ascent)')
        end

    elseif isequal(task,'stairdescent')
        filename = 'stairs_1_down_segmented.mat';
        fullpath = strcat(rootpath,filename);
        data = load(fullpath);
        if isequal(joint,'ankle')
            theta = data.angle.avg_r.sig_mu.ankle_angle_r;
            jointTorque = data.moment_filt.avg_r.sig_mu.ankle_angle_r_moment;
        elseif isequal(joint,'knee')
            theta = data.angle.avg_r.sig_mu.knee_angle_r;
            jointTorque = data.moment_filt.avg_r.sig_mu.knee_angle_r_moment;
        elseif isequal(joint,'hip')
            theta = data.angle.avg_r.sig_mu.hip_flexion_r;
            jointTorque = data.moment_filt.avg_r.sig_mu.hip_flexion_r_moment;
        else
            disp('Invalid GT2023 Joint Selected (Stair Descent)')
        end

    elseif isequal(task,'rampascent')
        filename = 'incline_walk_1_up5_segmented.mat';
        fullpath = strcat(rootpath,filename);
        data = load(fullpath);
        if isequal(joint,'ankle')
            theta = data.angle.avg_l.sig_mu.ankle_angle_l;
            jointTorque = data.moment_filt.avg_l.sig_mu.ankle_angle_l_moment;
        elseif isequal(joint,'knee')
            theta = data.angle.avg_l.sig_mu.knee_angle_l;
            jointTorque = data.moment_filt.avg_l.sig_mu.knee_angle_l_moment;
        elseif isequal(joint,'hip')
            theta = data.angle.avg_l.sig_mu.hip_flexion_l;
            jointTorque = data.moment_filt.avg_l.sig_mu.hip_flexion_l_moment;
        else
            disp('Invalid GT2023 Joint Selected (Ramp Ascent, 5 degrees)')
        end

    elseif isequal(task,'rampdescent')
        filename = 'incline_walk_1_down5_segmented.mat';
        fullpath = strcat(rootpath,filename);
        data = load(fullpath);
        if isequal(joint,'ankle')
            theta = data.angle.avg_r.sig_mu.ankle_angle_r;
            jointTorque = data.moment_filt.avg_r.sig_mu.ankle_angle_r_moment;
        elseif isequal(joint,'knee')
            theta = data.angle.avg_r.sig_mu.knee_angle_r;
            jointTorque = data.moment_filt.avg_r.sig_mu.knee_angle_r_moment;
        elseif isequal(joint,'hip')
            theta = data.angle.avg_r.sig_mu.hip_flexion_r;
            jointTorque = data.moment_filt.avg_r.sig_mu.hip_flexion_r_moment;
        else
            disp('Invalid GT2023 Joint Selected (Ramp Descent, 5 degrees)')
        end
    elseif isequal(task,'squat25lb')
        filename = 'squats_1_25lbs_segmented.mat';
        fullpath = strcat(rootpath,filename);
        data = load(fullpath);
        if isequal(joint,'ankle')
            theta = data.angle.avg_r.sig_mu.ankle_angle_r;
            jointTorque = data.moment_filt.avg_r.sig_mu.ankle_angle_r_moment;
        elseif isequal(joint,'knee')
            theta = data.angle.avg_r.sig_mu.knee_angle_r;
            jointTorque = data.moment_filt.avg_r.sig_mu.knee_angle_r_moment;
        elseif isequal(joint,'hip')
            theta = data.angle.avg_r.sig_mu.hip_flexion_r;
            jointTorque = data.moment_filt.avg_r.sig_mu.hip_flexion_r_moment;
        else
            disp('Invalid GT2023 Joint Selected (Ramp Descent, 5 degrees)')
        end
    else
        disp('Invalid GT2023 Task Selected')
    end

    %convert deg2rad
    theta = theta*deg2rad;
    % resample data for consistent length between tasks (not guaranteed
    % with GT dataset)

    % Thanks to Ross Cortino for showing how to do this a better way than
    % the 'resample' function!
    newsamples = 250;
    x_orig = 1:length(theta);
    x_new = linspace(1,length(theta),newsamples);

    theta = interp1(x_orig,theta,x_new);
    jointTorque = interp1(x_orig,jointTorque,x_new);
    
    %transpose outputs into columns
    theta = theta';
    jointTorque = -jointTorque'; %###SIGN FLIP

    %set time step, calculate stridetime vector
    dt = 1/length(theta); %assume all gaits are periodic at 1 Hz
    strideduration = dt*(length(theta(:,1))-1);
    stridetime = (0:dt:strideduration)'; %[sec]
    


    %shift ankle angles to start trajectory at zero
%     theta = (theta-theta(1));

    %calculate derivatives
    theta_d = ddt_open(theta,dt);
    theta_dd = ddt_open(theta_d,dt);


    %any other processing that must be done to all GT2023 data goes here

%% GT2023 MEAN STRIDE ACROSS SUBJECTS (VIA GAITCOMPARISON.M PIPELINE)
elseif isequal(dataset,'GT2023means_mat')
    if isequal(task,'normalwalk1x2')  %normal walking (1.2 m/s)
        if isequal(joint,'ankle')
            taskdata = load("ankle_normalwalk1x2_allsubavgs.mat");
            theta = taskdata.theta_ankle_allsubavg;
            jointTorque = taskdata.jointTorque_ankle_allsubavg;
        elseif isequal(joint,'knee')
            taskdata = load("knee_normalwalk1x2_allsubavgs.mat");
            theta = taskdata.theta_knee_allsubavg;
            jointTorque = taskdata.jointTorque_knee_allsubavg;
        elseif isequal(joint,'hip')
            taskdata = load("hip_normalwalk1x2_allsubavgs.mat");
            theta = taskdata.theta_hip_allsubavg;
            jointTorque = taskdata.jointTorque_hip_allsubavg;
        else
            disp('Invalid GT2023means Joint Selected (Normal Walk 1.2 m/s)')
        end
        
    elseif isequal(task,'stairascent') %switched from R to L due to NaN in torque array (last two samples)
        if isequal(joint,'ankle')
            taskdata = load("ankle_stairascent_allsubavgs.mat");
            theta = taskdata.theta_ankle_allsubavg;
            jointTorque = taskdata.jointTorque_ankle_allsubavg;
        elseif isequal(joint,'knee')
            taskdata = load("knee_stairascent_allsubavgs.mat");
            theta = taskdata.theta_knee_allsubavg;
            jointTorque = taskdata.jointTorque_knee_allsubavg;
        elseif isequal(joint,'hip')
            taskdata = load("hip_stairascent_allsubavgs.mat");
            theta = taskdata.theta_hip_allsubavg;
            jointTorque = taskdata.jointTorque_hip_allsubavg;
        else
            disp('Invalid GT2023means Joint Selected (Stair Ascent)')
        end

    elseif isequal(task,'stairdescent')
        if isequal(joint,'ankle')
            taskdata = load("ankle_stairdescent_allsubavgs.mat");
            theta = taskdata.theta_ankle_allsubavg;
            jointTorque = taskdata.jointTorque_ankle_allsubavg;
        elseif isequal(joint,'knee')
            taskdata = load("knee_stairdescent_allsubavgs.mat");
            theta = taskdata.theta_knee_allsubavg;
            jointTorque = taskdata.jointTorque_knee_allsubavg;
        elseif isequal(joint,'hip')
            taskdata = load("hip_stairdescent_allsubavgs.mat");
            theta = taskdata.theta_hip_allsubavg;
            jointTorque = taskdata.jointTorque_hip_allsubavg;
        else
            disp('Invalid GT2023means Joint Selected (Stair Descent)')
        end

    elseif isequal(task,'rampascent')
        if isequal(joint,'ankle')
            taskdata = load("ankle_rampascent_allsubavgs.mat");
            theta = taskdata.theta_ankle_allsubavg;
            jointTorque = taskdata.jointTorque_ankle_allsubavg;
        elseif isequal(joint,'knee')
            taskdata = load("knee_rampascent_allsubavgs.mat");
            theta = taskdata.theta_knee_allsubavg;
            jointTorque = taskdata.jointTorque_knee_allsubavg;
        elseif isequal(joint,'hip')
            taskdata = load("hip_rampascent_allsubavgs.mat");
            theta = taskdata.theta_hip_allsubavg;
            jointTorque = taskdata.jointTorque_hip_allsubavg;
        else
            disp('Invalid GT2023means Joint Selected (Ramp Ascent, 10 degrees)')
        end

    elseif isequal(task,'rampdescent')
        if isequal(joint,'ankle')
            taskdata = load("ankle_rampdescent_allsubavgs.mat");
            theta = taskdata.theta_ankle_allsubavg;
            jointTorque = taskdata.jointTorque_ankle_allsubavg;
        elseif isequal(joint,'knee')
            taskdata = load("knee_rampdescent_allsubavgs.mat");
            theta = taskdata.theta_knee_allsubavg;
            jointTorque = taskdata.jointTorque_knee_allsubavg;
        elseif isequal(joint,'hip')
            taskdata = load("hip_rampdescent_allsubavgs.mat");
            theta = taskdata.theta_hip_allsubavg;
            jointTorque = taskdata.jointTorque_hip_allsubavg;
        else
            disp('Invalid GT2023mean Joint Selected (Ramp Descent, 10 degrees)')
        end
    elseif isequal(task,'squat25lb')
        if isequal(joint,'ankle')
            taskdata = load("ankle_squat25lb_allsubavgs.mat");
            theta = taskdata.theta_ankle_allsubavg;
            jointTorque = taskdata.jointTorque_ankle_allsubavg;
        elseif isequal(joint,'knee')
            taskdata = load("knee_squat25lb_allsubavgs.mat");
            theta = taskdata.theta_knee_allsubavg;
            jointTorque = taskdata.jointTorque_knee_allsubavg;
        elseif isequal(joint,'hip')
            taskdata = load("hip_squat25lb_allsubavgs.mat");
            theta = taskdata.theta_hip_allsubavg;
            jointTorque = taskdata.jointTorque_hip_allsubavg;
        else
            disp('Invalid GT2023means Joint Selected (25 lb lift/lower)')
        end
    else
        disp('Invalid GT2023mean Task Selected')
    end

    %convert deg2rad
%     theta = theta*deg2rad;
    % resample data for consistent length between tasks (not guaranteed
    % with GT dataset)

    % Thanks to Ross Cortino for showing how to do this a better way than
    % the 'resample' function!
    newsamples = 250;
    x_orig = 1:length(theta);
    x_new = linspace(1,length(theta),newsamples);

    theta = interp1(x_orig,theta,x_new);
    jointTorque = interp1(x_orig,jointTorque,x_new);
    
    %transpose outputs into columns
    theta = theta';
    jointTorque = jointTorque'; 

    %set time step, calculate stridetime vector
    dt = 1/length(theta); %assume all gaits are periodic at 1 Hz
    strideduration = dt*(length(theta(:,1))-1);
    stridetime = (0:dt:strideduration)'; %[sec]
    


    %shift ankle angles to start trajectory at zero
%     theta = (theta-theta(1));

    %calculate derivatives
    theta_d = ddt_open(theta,dt);
    theta_dd = ddt_open(theta_d,dt);


    %any other processing that must be done to all GT2023 data goes here

    %circ shift 
%     theta = circshift(theta,length(theta)/2);
%     jointTorque = circshift(jointTorque,length(jointTorque)/2);
else
    disp('Invalid Dataset Selected')
end



end

