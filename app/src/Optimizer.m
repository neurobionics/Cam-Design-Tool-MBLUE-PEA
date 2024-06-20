function [X,viable,surfpoints,thetamin,thetamax] = ... 
Optimizer(inputJoint,inputTasks,inputDataset,inputTaskWeights,...
inputAssistFrac,inputActuator,inputSubnum,inputX0,multiplotAssistFrac,springparams,figTF)
close all
run("definecolorset.m") %pick colors for figures

tasklist = inputTasks;

subnum = inputSubnum; %placeholder input to gait retrieval function

datasetlist = inputDataset;

taskweights = inputTaskWeights;

assistFractionList = inputAssistFrac;

tasks = length(assistFractionList);

%Subject Mass
subMass = 75; %[kg]

%Joint Selection
joint = inputJoint; %choose 'ankle', 'knee', or 'hip'
%Actuator Selection
actuator = inputActuator;


X0 = inputX0;

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

assistFractionNEW = multiplotAssistFrac;
exoTorque = assistFractionNEW*subMass*jointTorque;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     %re-run cost function with optimal cx vector  (weight is one now)
[I2Rloss,I2R,residualTorque,cubicTorque,IVenergyUse,IVpow,current] = I2Rcostfunc(cxvec,motor_params,theta,theta_d,theta_dd,stridetime,exoTorque,1);

% Baseline Case (no parallel elastic element)
[I2RlossBase,I2RBase,residualTorqueBase,quadraticTorqueBase,IVenergyUseBase,IVpowBase,currentBase] = I2Rcostfunc([0 0 0 0 0 0 0],motor_params,theta,theta_d,theta_dd,stridetime,exoTorque,1);

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
% springparams.Rate_lbs_in_ = 61.183;
% springparams.FreeLength_in_ = 3.858;
% springparams.MaxDeflection_in_ = 1.17;
% springparams.MaxEPE_J_ = 4.7314;

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
    [viable, surfpoints] = cam_surf_gen(cxvec,thetamin,thetamax,...
        springparams,preloadenergy,camsaveY1N0,figTF);
    PEF = PEF + .01;

    if PEF > 1
        break
    end
end

%% Make Polar Plot for (R, Psi)
open("Rpsi_debug.mat");
if figTF
    f1 = figure;
    polarplot(ans.psi,ans.R)
    hold on
    fullcircpsi = linspace(-2*pi,2*pi,1000);
    fullcircR = .098/2*ones(size(fullcircpsi));
    polarplot(fullcircpsi,fullcircR,'r')
    
    camplot.fullcircR = fullcircR;
    camplot.fullcircPsi = fullcircpsi;
    camplot.camR = ans.R;
    camplot.camPsi = ans.psi;
end
%% Find energy savings for each task with the final setup

% If the assistance fraction is kept at the design level
if PEF<=1
    for i = 1:tasks
        EnergySaved(i) = 100*(1-trapz(stridetime(:,i),I2R(:,i))/trapz(stridetime(:,i),I2RBase(:,i)));
    end
end

% If the assistance fraction is changed, but the designed cam setup is used
assistFractionNEW = multiplotAssistFrac;

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
if figTF
    hfig = figure;  % save the figure handle in a variable
    
    n = 2; %downsample factor
    for i = 1:tasks
        %subplot 1: plot torque
        subplot(4,3,(i-1)*3+2)
        plot(downsample(stridetime(:,i),n),downsample(exoTorque(:,i),n),'color',lite)
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
        plot(downsample(theta(:,i),n),downsample(exoTorque(:,i),n),'color',lite) %total exo torque as fcn of angle
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
        ylabel('I^2R Power (W)')
        if i==1
            leg3 = legend('Baseline (QDD)','PEA');
            set(leg3,'Box','off','Location','south','Orientation','horizontal')
        elseif i == 4
            xlabel('Stride Time (sec)')
        end
        
    end
    
    %% Multiplot Figure Property Setup
    fname = 'myfigure';
    picturewidth = 20; % set this parameter and keep it forever
    hw_ratio = 0.65; % feel free to play with this ratio
    set(findall(hfig,'-property','FontSize'),'FontSize',8) % adjust fontsize to your document
    set(findall(hfig,'-property','LineWidth'),'LineWidth',1.5) % adjust linewidth
    
    set(findall(hfig,'-property','Box'),'Box','off') % optional
    set(findall(hfig,'-property','Interpreter'),'Interpreter','latex') 
    set(findall(hfig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
    set(hfig,'Units','centimeters','Position',[3 3 picturewidth hw_ratio*picturewidth])
    pos = get(hfig,'Position');
    set(hfig,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])
    set(leg3,'Position',[0.6335    0.9491    0.3443    0.0504]);
    set(leg1,'Position',[0.0340    0.9491    0.5959    0.0504]);
end
X = cxvec;
end