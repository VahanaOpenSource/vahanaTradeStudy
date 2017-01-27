% This script attempts to minimize direct operating costs (as a proxy for
% ticket price) for different small electric vehicle configurations.
%
% Design variables include high level parameters and variables used to
% converge to a closed solution.
%
% Each vehicle has many assumptions that are described in the different
% model functions.
%

function [] = sizingTradeStudy()

clear
clc

% Constants
lb2kg = 0.453592;

% User inputs
plotOn = true;
ranges = (10:10:200) * 1e3; % m
payload = 250 * lb2kg; % kg

% Reset random number generator for repeatability
rng('default');

% Optimization options
options = optimset('display','notify','Algorithm','interior-point','ScaleProblem','obj-and-constr','MaxIter',5e3,'MaxFunEvals',2e4,'FinDiffType','central');

% Number of random optimization restarts to try since there may be local minima
nRestart = 3;

% Flags indicating that previous range has converged for a given config
exitflagH = 1; % Helicopter
exitflagT = 1; % Tilt-wing

% Function / Objective calls
xLast = []; % Last place computeall was called
myf = []; % Use for objective at xLast
myc = []; % Use for nonlinear inequality constraint


%% Loop over design ranges
for i = 1:length(ranges)
    
    disp(['Range = ',num2str(ranges(i)/1000),' km']);
    range = ranges(i);
    
    %% Helicopter
    % Design variables (rotor radius, cruise speed, battery mass, motor mass, takeoff mass)
    vehicle = 'helicopter';
    disp('Helicopter')
    
    % Initial guess for first iteration, start with previous solution for
    % future iterations
    if i == 1
        x0 = [3, 50, 117, 30, 650];
    else 
        x0 = xH;
    end
    
    % Design variable bounds
    lb = [1, 30, 10,  1, 100];
    ub = [10, 80, 999, 999, 9999];
    
    if exitflagH > 0 % Don't run if it hasn't converged at the previous speed
        
        [xH,fval,exitflagH,output] = fmincon(...
            @(x) objfun(x,vehicle,range,payload),...
            x0,[],[],[],[],lb,ub,...
            @(x) confun(x,vehicle,range,payload),...
            options);
        
        % Run Multistart cases
        for k = 1:nRestart
            [xTmp,fvalTmp,exitflagTmp,outputTmp] = fmincon(...
                @(x) objfun(x,vehicle,range,payload),...
                lb + rand(size(x0)).*(ub-lb),[],[],[],[],lb,ub,...
                @(x) confun(x,vehicle,range,payload),...
                options);
            
            % Save if previous solution didn't converge, or if this is a
            % better solution
            if exitflagH < 1 || (fvalTmp < fval && exitflagTmp == 1)
                xH = xTmp;
                fval = fvalTmp;
                exitflagH = exitflagTmp;
                output = outputTmp;
            end
        end
        
        if exitflagH > 0
            % Get optimal states
            rPropHelicopter(i) = xH(1); %#ok
            VHelicopter(i) = xH(2); %#ok
            mBattery = xH(3);
            mMotors = xH(4);
            mtow = xH(5);
            
            % Compute energy used for simple mission
            [EHelicopter(i),flightTimeHelicopter(i),hoverOutputHelicopter(i),cruiseOutputHelicopter(i)] = simpleMission(vehicle,rPropHelicopter(i),VHelicopter(i),mtow*9.8,range); %#ok
            
            % Compute energy for reserve missoin
            [EReserveHelicopter(i),~,~,~,loiterOutputHelicopter(i)] = reserveMission(vehicle,rPropHelicopter(i),VHelicopter(i),mtow*9.8,range); %#ok
            
            % Compute weight estimate
            massHelicopter(i) = configWeight(vehicle,rPropHelicopter(i),mBattery,mMotors,mtow,hoverOutputHelicopter(i),cruiseOutputHelicopter(i),payload); %#ok
            
            % Compute operating cost
            CHelicopter(i) = operatingCost(vehicle,rPropHelicopter(i),flightTimeHelicopter(i),EHelicopter(i),massHelicopter(i),cruiseOutputHelicopter(i)); %#ok
            
            disp('Min Cost Helicopter');
            disp('===================');
            disp(['Operating Cost [$]: ',num2str(fval)]);
            disp(['  Rotor Radius [m]: ',num2str(xH(1))]);
            disp(['Cruise Speed [m/s]: ',num2str(xH(2))]);
            disp([' Battery Mass [kg]: ',num2str(xH(3))]);
            disp(['   Motor Mass [kg]: ',num2str(xH(4))]);
            disp(['         MTOW [kg]: ',num2str(xH(5))]);
        else
            disp('Optimization failed for Helicopter!');
            disp(output);
        end
    end
    
    %% Tilt-Wing Multirotor Configuration
    % Design variables (rotor radius, speed, battery mass, motor mass, mtow)
    vehicle = 'tiltwing';
    disp('Tilt-Wing')
    
    % Initial guess for first iteration, start with previous solution for
    % future iterations
    if i == 1
        x0 = [1,   50, 117, 30, 650];
    else
        x0 = xT;
    end
    
    % Design variable bounds
    lb = [0.3, 1.3 * 35, 10,  1, 100]; % Min cruise at 1.3 * VStall
    ub = [2,   80, 999, 999, 9999];
    
    if exitflagT > 0 % Don't run if it hasn't converged at the previous speed
        
        [xT,fval,exitflagT,output] = fmincon(...
            @(x) objfun(x,vehicle,range,payload),...
            x0,[],[],[],[],lb,ub,...
            @(x) confun(x,vehicle,range,payload),...
            options);
        
        % Run Multistart cases to try again
        for k = 1:nRestart
            [xTmp,fvalTmp,exitflagTmp,outputTmp] = fmincon(...
                @(x) objfun(x,vehicle,range,payload),...
                lb + rand(size(x0)).*(ub-lb),[],[],[],[],lb,ub,...
                @(x) confun(x,vehicle,range,payload),...
                options);
            
            % Save if previous solution didn't converge, or if this is a
            % better solution
            if exitflagT < 1 || (fvalTmp < fval && exitflagTmp == 1)
                xT = xTmp;
                fval = fvalTmp;
                exitflagT = exitflagTmp;
                output = outputTmp;
            end
        end
        
        if exitflagT > 0
            
            disp('Min Cost Tilt-Wing Design');
            disp('=========================');
            disp(['Operating Cost [$]: ',num2str(fval)]);
            disp(['  Rotor Radius [m]: ',num2str(xT(1))]);
            disp(['Cruise Speed [m/s]: ',num2str(xT(2))]);
            disp([' Battery Mass [kg]: ',num2str(xT(3))]);
            disp(['   Motor Mass [kg]: ',num2str(xT(4))]);
            disp(['         MTOW [kg]: ',num2str(xT(5))]);
            
            rPropTiltWing(i) = xT(1); %#ok
            VTiltWing(i) = xT(2); %#ok
            mBattery = xT(3);
            mMotors = xT(4);
            mtow = xT(5);
            
            % Compute energy used for simple mission
            [ETiltWing(i),flightTimeTiltWing(i),hoverOutputTiltWing(i),cruiseOutputTiltWing(i)] = simpleMission(vehicle,rPropTiltWing(i),VTiltWing(i),mtow*9.8,range); %#ok
            
            % Compute energy for reserve missoin
            [EReserveTiltWing(i),~,~,~,loiterOutputTiltWing(i)] = reserveMission(vehicle,rPropTiltWing(i),VTiltWing(i),mtow*9.8,range); %#ok
            
            % Weight estimate
            massTiltWing(i) = configWeight(vehicle,rPropTiltWing(i),mBattery,mMotors,mtow,hoverOutputTiltWing(i),cruiseOutputTiltWing(i),payload); %#ok
            
            % Compute operating cost
            CTiltWing(i) = operatingCost(vehicle,rPropTiltWing(i),flightTimeTiltWing(i),ETiltWing(i),massTiltWing(i),cruiseOutputTiltWing(i)); %#ok
            
        else
            disp('Optimization failed for Tilt-Wing!');
            disp(output);
        end
    end   
end

save('tradeStudyResults.mat');

if plotOn
    plotTradeResults();
end

    % Compute objective and constraint values in the same function. Only
    % re-run if the inputs have changed.
    function y = objfun(x,vehicle,range,payload)
        
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc] = computePerformance(x,vehicle,range,payload);
            xLast = x;
        end
        
        % Now compute objective function
        y = myf;
    end

    function [c,ceq] = confun(x,vehicle,range,payload)
        
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc] = computePerformance(x,vehicle,range,payload);
            xLast = x;
        end
        
        % Now compute constraint functions
        c = myc; % In this case, the computation is trivial
        ceq = [];
    end
end
