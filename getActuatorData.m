function [motor_params] = getActuatorData(actuatorname)
%% Converstion Factors
rev2rad = 2*pi;
min2sec = 1/60;
deg2rad = pi/180;
in2m = .0254;
lb2n = 4.4482216153;
%% Bring in actuator

if isequal(actuatorname,'AK80-9')
    % Assign Motor Thermal Properties
%     % https://github.com/UM-LoCoLab/elderly_exo_study/blob/main/device_side/thermal_model.py
%     C_w =  0.20*81.46202695970649; %thermal capacitance for windings
%     R_WH =  1.0702867186480716; %thermal resistance between windings and case
%     C_h =  512.249065845453; %thermal capacitance for case
%     R_HA =  1.9406620046327363; %thermal resistance between case and ambient
%     alpha = 0.393*1/100; %Pure copper. Taken from thermalmodel3.py
%     R_T_0 = 65;% temp at which resistance was measured
%     Rphase_0 = .376; % emirical, from the computed resistance (q-axis voltage/ q-axis current). Ohms
%     T0=22; %starting temp

    % AK80-9 properties
    %https://store.tmotor.com/goods-982-AK80-9.html
    motor_params.name = 'AK80-9';
    motor_params.bm = (-6/50)/(rev2rad/min2sec); %[Nm.s/rad] 
    motor_params.Rterm = .170; %[Ohms] "phase to phase resistance"
    motor_params.Lterm = .000057; %[H] phase to phase inductance
    motor_params.T_rated = 9; %[Nm]
    motor_params.T_peak = 18; %[Nm]
    motor_params.backdrive = 0.51; %[Nm]
    motor_params.backlash = 0.19; %[deg]
    motor_params.kt = 0.105; %[Nm/A]
    motor_params.kv = 100; %[rpm/V]
    motor_params.V_rated = 48; %[V]
    motor_params.GR = 9;
    motor_params.inertia = 607; %[gcm2]
    motor_params.mass = 485; %[g]
    
    motor_params.Le = .5*motor_params.Lterm; %delta winding conversion from line-line to effective inductance

    motor_params.ratedspeed = 390; %[rpm]
    motor_params.thickness = 38.5; %[mm]
    motor_params.diameter = 98; %[mm]
    
    motor_params.price = 579.9; %[USD]
    % motor_params.torquespeed_diamond = [-motor_params.ratedspeed, 0;
    
    motor_params.inertia = motor_params.inertia*1e-7; %[kg.m2]
    motor_params.Rphase = motor_params.Rterm*3/2;
else
    disp('Invalid Actuator Selected')
end

end