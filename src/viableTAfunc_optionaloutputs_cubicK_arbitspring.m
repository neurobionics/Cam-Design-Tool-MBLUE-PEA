function [isthiscamviable] = viableTAfunc_optionaloutputs_cubicK_arbitspring(X,thetamin,thetamax,springparams,preloadenergy,camsaveY1N0)
%VIABLETAFUNC This function takes in a design variables for the desired spring stiffness function to render,
% the range over which it must be rendered, the parameters of the spring to use
% It returns a binary value indicating whether or not a viable cam was
% found, and writes that cam's geometry to a file for use in CAD

    cx3d = X(1);
    cx2d = X(2);
    cx1d = X(3);
    cx3p = X(4);
    cx2p = X(5);
    cx1p = X(6);
    z = X(7);

    isthiscamviable = 0;
    theta = (linspace(thetamin,thetamax,10000)); %create the array of points from "zero" to the ROM limit for this cycle

    %% Set up the torque angle profiles

    funcP = @(th) cx3p.*(th-z).^3 + cx2p.*(th-z).^2 + cx1p.*(th-z); %make the function for k = f(theta)
    funcD = @(th) cx3d.*(th-z).^3 + cx2d.*(th-z).^2 + cx1d.*(th-z); %make the function for k = f(theta)
    
    tauP = funcP(theta); %use the @ func on the ROM vector
    tauD = funcD(theta); %use the @ func on the ROM vector
    tau = zeros(1,length(theta));
    for i=1:length(theta)
        if theta(i)<=z
            tau(i) = tauP(i);
        else
            tau(i) = tauD(i);
        end
    end
    i=1;
    while theta(i)<z %segment the indeces into pos and neg ROM
        i = i+1;
    end

    theta_negvec = [theta(1:i-1),z]; %neg lim->zero
    theta_negvec = flip(theta_negvec); % FLIP to be zero->neg CRUCIAL FOR INTEGRATION STEP 
    zeropointY = interp1(theta(i-1:i),tau(i-1:i),z);
    theta_posvec = [z,theta(i:end)];
    torque1  = [tau(1:i-1),zeropointY];
    torque1 = flip(torque1); % FLIP to be zero->neg CRUCIAL FOR INTEGRATION STEP 
    torque2 = [zeropointY,tau(i:end)];
        
    %Take preload energy, calculate corresponding spring preload

    g0 = (2*preloadenergy/springparams.Kspring)^.5; % EPE=(1/2)kx^2 --> x = (2EPE/k)^.5
    preloadEPE = .5*springparams.Kspring*g0^2;
    maxAllowableEPE = .5*springparams.Kspring*springparams.compressLimit^2;

    
    % Define geometric parameters of the module's CAD 
    moduleparams.L_carriage = 0.0220; %[m]
%     moduleparams.L_jointaxistospringground = 0.14638; %[m] update this if cad changes ###I CAN CHANGE THIS WITH SPACERS
    moduleparams.r_follower = 0.0065; %[m] follower bearing radius
  
    basecamrad = .051+moduleparams.r_follower; % limited by motor diameter to retain packing factor
    
    % ### DEBUG BASED ON SPRINGPARAMETERMATGENERATOR.M
%     moduleparams.L_jointaxistospringground = (springparams.L_spring - g0) + moduleparams.L_carriage + moduleparams.r_follower + basecamrad;
    moduleparams.L_jointaxistospringground = basecamrad + (springparams.L_spring-g0) + moduleparams.L_carriage; 

    %% PLANTARFLEXION (OR, NEGATIVE ROM)
    g = [];
    
    intMadtheta1 = cumtrapz(theta_negvec,torque1); %cumulative integral of Ma*dtheta
    WorkInSpring1 = preloadEPE + intMadtheta1;
    for i = 1:length(theta_negvec)
        %deflection of spring (including preload)
        g(i) = -g0 + sqrt(g0^2 + (2/springparams.Kspring)*intMadtheta1(i));  %shepherd rouse TNSRE17
        %instantaneous cam radius
        r1(i) = g(i) + basecamrad; %####
    end
    g1 = g;
    %% DORSIFLEXION (OR, POSITIVE ROM)
    intMadtheta2 = cumtrapz(theta_posvec,torque2); %cumulative integral of Ma*dtheta
    WorkInSpring2 = preloadEPE + intMadtheta2;

    for i = 1:length(theta_posvec)
        %find k for compression g via lookup table for spring here if nonlinear
        
        %get deflection values using virtual work principle
        g(i) = -g0 + sqrt(g0^2 + (2/springparams.Kspring)*intMadtheta2(i));  %shepherd rouse TNSRE17

        g2(i) = g(i); %move to new variable for future concatination
        
        %add the base cam radius (the cam radius for which the zero-radius follower
        %bearing would be tangent to the circular cam when spring is at
        %equilibrium length)
        r2(i) = g(i)+basecamrad;
    
    end
    
    %% Combine DF and PF, plot
    
    ankletheta_cam = [flip(theta_negvec),theta_posvec(2:end)]; %concatinate theta arrays to form full cam ROM
    
    rvec = [flip(r1),r2(2:end)]; %INSTANTANEOUS CAM RADIUS (BEARING RAD STILL ZERO)
    gvec = [flip(g1),g2(2:end)]; %INSTANTANEOUS DEFLECTION MAGITUDE (BEARING RAD STILL ZERO)
  

    %% Convert polar coordinates to cartesian
    rvecX = zeros(length(rvec),1); %preallocate
    rvecY = zeros(length(rvec),1); %preallocate
    basecamradX = zeros(length(rvec),1); %preallocate
    basecamradY = zeros(length(rvec),1); %preallocate

    for j = 1:length(rvec)
        rvecY(j) = rvec(j)*cos(ankletheta_cam(j));
        rvecX(j) = rvec(j)*sin(ankletheta_cam(j));
        
        basecamradY(j) = basecamrad*cos(ankletheta_cam(j));
        basecamradX(j) = basecamrad*sin(ankletheta_cam(j));
        
    end
    %% normal line approach for finding cam surface from deflection curve
    for i = 1:length(rvecY)-1
        [xy_Raway_alongnorm(i,1),xy_Raway_alongnorm(i,2)] = CWvectorRotate90(rvecX(i),rvecY(i),rvecX(i+1),rvecY(i+1),moduleparams.r_follower);
    end

    %% Optional saving of this cam trajectory
    if camsaveY1N0 == 1
%                 version for ufiltered surf
        zvec = zeros(length(xy_Raway_alongnorm(:,1)),1);
        camsurfacepoints = cat(2,xy_Raway_alongnorm,zvec);
        camsurfacepoints = camsurfacepoints*1000; %convert to mm
        camsurfacepointsSORTED = sortrows(camsurfacepoints); %sort by x index 
        writematrix(camsurfacepoints,'cam_surface_pointsDEBUG.txt')
        
        %save (R, psi) parameterization
        R = rvec;
        psi = ankletheta_cam;
        save("Rpsi_debug.mat","R","psi","g0","springparams","moduleparams","X")
        
    end
    %% DETERMINE WHETHER THE GENERATED SURFACE IS VIABLE
        
    % check for self-intersections using InterX
    % source: https://www.mathworks.com/matlabcentral/fileexchange/22441-curve-intersections
    intersections = InterX(xy_Raway_alongnorm');
    
    maxCompress=max(abs(gvec+g0));
    
    %check for excessive compression (beyond max deflection limit)
    if maxCompress> springparams.compressLimit
        compressionPass = 0;
        disp('Non-Viable: Spring Deflected Beyond Limit')
    else
        compressionPass = 1;
    end
    
    %% Check for energy expenditure viability
    % get the cumulative values for integrating under torque-angle
    % curve from zero torque point to each limit
    excessiveCompression = 0;
    excessiveExtension = 0;
    if max(WorkInSpring1)>=maxAllowableEPE
        excessiveCompression = 1;
        disp('Excessive compression in region 1')
    end
    if max(WorkInSpring2)>=maxAllowableEPE
        excessiveCompression = 1;
        disp('Excessive compression in region 2')
    end
    if min(WorkInSpring1)<=0
        excessiveExtension = 1;
        disp('Insufficient preload due to region 1')
    end
    if min(WorkInSpring2)<=0
        excessiveExtension = 1;
        disp('Insufficient preload due to region 2')
    end 
    if ~isempty(intersections)
        disp('Non-viable cam due to intersections')
    end
    %% assess viability conditions, add or reject configuration
    if isempty(intersections) == 1 && compressionPass == 1 && ~excessiveExtension && ~excessiveCompression
        %  the surface is viable, add this set of cam and hardware parameters
        isthiscamviable = 1;
        
        figure
        title('Cam Surface Profiles')
        plot(xy_Raway_alongnorm(:,1),xy_Raway_alongnorm(:,2),'or','LineWidth',2)
        axis equal
        hold on
        % f(0,-mountdist,'ok')
        plot(0,0,'ok')
  

        plot(rvecX,rvecY,'ob')


        xlabel('X Dimension (m)')
        ylabel('Y Dimension (m)') 
        leg1=legend('Cam Surface','Rotation Center','Progenetor Curve');
        set(leg1,'Box','off','Location','best','Orientation','vertical')
 

    end
    
    if isthiscamviable == 0
        disp('Exiting Cam Gen: No viable profile found for given inputs')
    end

end