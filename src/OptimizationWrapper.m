clc
clear all
close all

run("definecolorset.m") %pick colors for figures

%% Choose Your Inputs!

tasks = 4;

%Ambulation Task Selection

% GT2023 Task Options
% normalwalk1x2, stairascent, stairdescent, rampascent, rampdescent,
% squat25lb

tasklist{1} = 'squat25lb';
tasklist{2} = 'normalwalk1x2';
tasklist{3} = 'rampascent';
tasklist{4} = 'rampdescent';

subnum = 'AB02'; %placeholder input to gait retrieval function

datasetlist{1} = 'GT2023means_mat';
datasetlist{2} = 'GT2023means_mat';
datasetlist{3} = 'GT2023means_mat';
datasetlist{4} = 'GT2023means_mat';

taskweights = [.3 .5 .1 .1]; %must add up to 1.0

assistFractionList(1) = 0.08;
assistFractionList(2) = 0.1;
assistFractionList(3) = 0.1;
assistFractionList(4) = 0.1;

%Subject Mass
subMass = 75; %[kg]

%Joint Selection
joint = 'knee'; %choose 'ankle', 'knee', or 'hip'
%Actuator Selection
actuator = 'AK80-9';



%% Cubic Coeffs (Initial Config)
cx3d = -136;
cx2d = 57;
cx1d = 5;
cx3p = 2.4;
cx2p = 5.3;
cx1p = 5.46;
z = 0.001;

X0 = [cx3d,cx2d,cx1d,cx3p,cx2p,cx1p,z];

%constrain cx1p = cx1d
Aeq(1,:) = [0 0 1 0 0 -1 0];
beq(1,1) = 0;

%% Conversion Factors
rev2rad = 2*pi;
min2sec = 1/60;
deg2rad = pi/180;
in2m = .0254;
lb2n = 4.4482216153;

%% Bring In The Data

%Actuator Parameters (inertia,damping,resistance,etc)
[motor_params] = getActuatorData(actuator);

for i = 1:tasks
    dataset = datasetlist{i}; 
    task = tasklist{i};
    assistFraction = assistFractionList(i);
    %Gait Task Parameters (kinematics, joint moments)
    [jointTorque(:,i), theta(:,i), theta_d(:,i), theta_dd(:,i),stridetime(:,i)] = getGaitData(dataset,joint,task,subnum);
   
    %% Calculate Remaining Cost Function Input(s) -- Desired Exo Torque
    
    exoTorque(:,i) = assistFraction*subMass*jointTorque(:,i);

end
%% Run FMINCON

% we are now inputting multiple tasks in, so the loss will be cumulative
fun = @(x)I2Rcostfunc(x,motor_params,theta,theta_d,theta_dd,stridetime,exoTorque,taskweights);

max_thetamin = max(min(theta));
min_thetamax = min(max(theta)); 

 % don't let transition between functions occur too close to ROM limit
big = 1e6; % large placeholder for 'unconstrained' coefficients
lb = [-big -big -big -big -big -big .99*max_thetamin];
ub = [big big big big big big .99*min_thetamax];

cxvec = fmincon(fun,X0,[],[],Aeq,beq,lb,ub);

%%%% ONLY USE THIS PORTION IF YOU WANT TO USE THE CAM DESIGNED ABOVE WITH A
%%%% NEW EXO CONTROL PROFILE *FOR PLOTTING*
%%%% OUTPUT OF ENERGYSAVED AND ENERGYSAVEDNEW ARE REDUNDANT WITH THIS

assistFractionNEW = 0.25;
exoTorque = assistFractionNEW*subMass*jointTorque;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     %re-run cost function with optimal cx vector  (weight is one now)
[I2Rloss,I2R,residualTorque,cubicTorque,IVenergyUse,IVpow,current] = I2Rcostfunc(cxvec,motor_params,theta,theta_d,theta_dd,stridetime,exoTorque,1);

% Baseline Case (no parallel elastic element)
[I2RlossBase,I2RBase,residualTorqueBase,quadraticTorqueBase,IVenergyUseBase,IVpowBase,currentBase] = I2Rcostfunc([0 0 0 0 0 0 0],motor_params,theta,theta_d,theta_dd,stridetime,exoTorque,1);

%% Subplots (Torque, Angle, Stiffness, I2R Loss) for optimal solution
for i = 1:tasks
    I2Rreductionpct(i) = (1-trapz(stridetime(:,i),I2R(:,i))/trapz(stridetime(:,i),I2RBase(:,i)))*100;

    f(i) = figure;
    task = tasklist{i};
    assistFraction = assistFractionList(i);

    %subplot 1: plot torque
    subplot(2,2,1)
    plot(stridetime(:,i),exoTorque(:,i),'k')
    hold on
    plot(stridetime(:,i),residualTorque(:,i),'r')
    plot(stridetime(:,i),cubicTorque(:,i),'g')
    xlabel('Stride Time (sec)')
    ylabel('Torque (Nm)')
    leg1=legend('Total Exo Torque','Active','Parallel Elastic');
    set(leg1,'Box','off','Location','north','Orientation','horizontal')
    % box("off")
    
%     %subplot 2: plot angle
%     subplot(2,2,2)
%     plot(stridetime(:,i),theta(:,i),'k')
%     xlabel('Stride Time (sec)')
%     ylabel('Angle (rad)')

    %subplot 2: plot current
    subplot(2,2,2)
    plot(stridetime(:,i),current(:,i),'k')
    xlabel('Stride Time (sec)')
    ylabel('Current (A)')

    %subplot 3: plot stiffness
    subplot(2,2,3)
    plot(theta(:,i),exoTorque(:,i),'k') %total exo torque as fcn of angle
    hold on
    plot(theta(:,i),residualTorque(:,i),'r')
    plot(theta(:,i),cubicTorque(:,i),'g')
    
    xlabel('Angle (rad)')
    ylabel('Torque (Nm)')
    leg2=legend('Total Exo Torque','Active','Parallel Elastic','Orientation','horizontal');
    set(leg2,'Box','off','Location','north')
    
    %subplot 4: power 
    subplot(2,2,4)
    plot(stridetime(:,i),I2RBase(:,i)) %Joule Heating power with no spring
    hold on
    plot(stridetime(:,i),I2R(:,i)) %Joule Heating power with with selected quadratic spring
    xlabel('Stride Time (sec)')
    ylabel('I^2R Power (W)')
    leg3 = legend('Baseline (QDD)','PEA');
    set(leg3,'Box','off','Location','northeast','Orientation','horizontal')
    
    titlestr = sprintf('(FMINCON Optimal Coeffs Over %i Tasks): %i kg, %i pct assist (cam design), ',tasks,subMass,assistFraction*100);
    
    %%%% FOR USE WITH ASSIST FRAC CHANGE BETWEEN CAM DESIGN AND FINAL
    %%%% CONTROLLER
    titlestr = sprintf('(FMINCON Optimal Coeffs Over %i Tasks): %i kg, %i pct assist (cam design), %i pct assist (exo controller), ',tasks,subMass,assistFraction*100,assistFractionNEW*100);
    
    titlestr = strcat(titlestr,joint,', ',task,', ',datasetlist{i});
    sgtitle(titlestr) 

    f(i).Position(3) = f(i).Position(3)*2.5; %widen figures
    
end
%% Make a cam with the optimizer-chosen coefficients
% springindexquery = 13; %sku 76327
% tablefile = 'Springs_mwcomponents_OD_0x50_0x60_FL_2_6_ends_CG.xlsx';

camsaveY1N0 = 1; %do we want to save cam XY points and (R,psi)?

%choose an input spring (sku 1815: max EPE among Table)
% springparams.Rate_lbs_in_ = 33;
% springparams.FreeLength_in_ = 5.38;
% springparams.MaxDeflection_in_ = 1.7;
% springparams.MaxEPE_J_ = 5.3877;

%choose an input spring (sku 76328: 3rd max EPE among Table; shorter free length, almost 2x stiffness)
springparams.Rate_lbs_in_ = 61.183;
springparams.FreeLength_in_ = 3.858;
springparams.MaxDeflection_in_ = 1.17;
springparams.MaxEPE_J_ = 4.7314;

%convert to SI
springparams.L_spring = springparams.FreeLength_in_*in2m;
springparams.compressLimit = springparams.MaxDeflection_in_*in2m;
springparams.Kspring = springparams.Rate_lbs_in_*lb2n/in2m;

viable = 0;
PEF = 0.01; %preload energy fraction

while viable == 0
    preloadenergy = springparams.MaxEPE_J_*PEF;
    thetamin = min(theta,[],"all");
    thetamax = max(theta,[],"all");
    viable = viableTAfunc_optionaloutputs_cubicK_arbitspring(cxvec,thetamin,thetamax,springparams,preloadenergy,camsaveY1N0);
    PEF = PEF + .01

    if PEF > 1
        break
    end
end

%% Make Polar Plot for (R, Psi)
open("Rpsi_debug.mat");
f1 = figure;
polarplot(ans.psi,ans.R)
hold on
fullcircpsi = linspace(-2*pi,2*pi,1000);
fullcircR = .098/2*ones(size(fullcircpsi));
polarplot(fullcircpsi,fullcircR,'r')

%% Find energy savings for each task with the final setup

% If the assistance fraction is kept at the design level
if PEF<=1
    for i = 1:tasks
        EnergySaved(i) = 100*(1-trapz(stridetime(:,i),I2R(:,i))/trapz(stridetime(:,i),I2RBase(:,i)));
    end
end

% If the assistance fraction is changed, but the designed cam setup is used
assistFractionNEW = 0.25;

for i=1:tasks
    exoTorqueNEW(:,i) = assistFractionNEW*subMass*jointTorque(:,i);
end

etnONELINE = assistFractionNEW*subMass*jointTorque;

%run cost function with optimal cx vector AND NEW EXO TORQUE (weight is one now)
[I2RlossNEW,I2RNEW,residualTorqueNEW,cubicTorqueNEW,IVenergyUseNEW,IVpowNEW,currentNEW] = I2Rcostfunc(cxvec,motor_params,theta,theta_d,theta_dd,stridetime,exoTorqueNEW,1);

% Baseline Case
[I2RlossBaseNEW,I2RBaseNEW,residualTorqueBaseNEW,cubicTorqueBaseNEW,IVenergyUseBaseNEW,IVpowBaseNEW,currentBaseNEW] = I2Rcostfunc([0 0 0 0 0 0 0],motor_params,theta,theta_d,theta_dd,stridetime,exoTorqueNEW,1);


for i = 1:tasks
    EnergySavedNEW(i) = 100*(1-trapz(stridetime(:,i),I2RNEW(:,i))/trapz(stridetime(:,i),I2RBaseNEW(:,i)));
    i2rEnergySpentJnew(i) = trapz(stridetime(:,i),I2RNEW(:,i));

    IV_EnergySavedNEW(i) = 100*(1-IVenergyUseNEW(:,i)/IVenergyUseBaseNEW(:,i));

end

%% Create tiled composite fig for paper
% Create a new composite figure
hfig = figure;  % save the figure handle in a variable

n = 2; %downsample factor
for i = 1:tasks
    %subplot 1: plot torque
    subplot(4,3,(i-1)*3+2)
    plot(downsample(stridetime(:,i),n),downsample(exoTorque(:,i),n),'color',light)
    hold on
    plot(downsample(stridetime(:,i),n),downsample(residualTorque(:,i),n),'color',med)
    plot(downsample(stridetime(:,i),n),downsample(cubicTorque(:,i),n),'color',dark)
    if i==1
        leg1=legend('Total Exo Torque','Active','Parallel Elastic');
        set(leg1,'Box','off','Location','south','Orientation','horizontal')        
    elseif i == 4
        xlabel('Stride Time (sec)')
    end
    ylabel('Torque (Nm)')    

    %subplot 3: plot stiffness
    subplot(4,3,(i-1)*3+1)
    plot(downsample(theta(:,i),n),downsample(exoTorque(:,i),n),'color',light) %total exo torque as fcn of angle
    hold on
    plot(downsample(theta(:,i),n),downsample(residualTorque(:,i),n),'color',med)
    plot(downsample(theta(:,i),n),downsample(cubicTorque(:,i),n),'color',dark)
    
    if i == 4
        xlabel('Angle (rad)')
    end
    ylabel('Torque (Nm)')

    %subplot 4: power 
    subplot(4,3,(i-1)*3+3)
    plot(downsample(stridetime(:,i),n),downsample(I2RBase(:,i),n),'color',dark) %Joule Heating power with no spring
    hold on
    plot(downsample(stridetime(:,i),n),downsample(I2R(:,i),n),'color',med) %Joule Heating power with with selected quadratic spring
    if i==1
        leg3 = legend('Baseline (QDD)','PEA');
        set(leg3,'Box','off','Location','south','Orientation','horizontal')
    elseif i == 3
        ylabel('Joule Heating Power (W)')
    elseif i == 4
        xlabel('Stride Time (sec)')
    end
    
end

%% Multiplot Figure Property Setup
fname = 'myfigure';
picturewidth = 20; % set this parameter and keep it forever
hw_ratio = 0.65; % feel free to play with this ratio
set(findall(hfig,'-property','FontSize'),'FontSize',13) % adjust fontsize to your document
set(findall(hfig,'-property','LineWidth'),'LineWidth',1.5) % adjust linewidth

set(findall(hfig,'-property','Box'),'Box','off') % optional
set(findall(hfig,'-property','Interpreter'),'Interpreter','latex') 
set(findall(hfig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
set(hfig,'Units','centimeters','Position',[3 3 picturewidth hw_ratio*picturewidth])
pos = get(hfig,'Position');
set(hfig,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])


%% Output Numeric Results For Paper

for i = 1:length(tasklist)
    msg = sprintf('Joule energy loss (now %1.3f J per stride) reduced by %2.3f pct for %s task, %1.1f assist fraction controller \n',i2rEnergySpentJnew(i),EnergySavedNEW(i),tasklist{i},assistFractionNEW);
    disp(msg)
end

for i = 1:length(tasklist)
    msg = sprintf('Electrical energy use (now %1.3f J per stride) reduced by %2.3f pct for %s task, %1.1f assist fraction controller \n',IVenergyUseNEW(i),IV_EnergySavedNEW(i),tasklist{i},assistFractionNEW);
    disp(msg)
end