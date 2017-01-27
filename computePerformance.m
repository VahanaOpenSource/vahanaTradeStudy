% Computes Optimization objective and constraints
%
% Inputs:
%  x       - design variales (prop/rotor radius [rProp], cruise speed [V],
%            battery mass [mBattery], motor mass [mMotors], max takeoff
%            mass [mtow])
%  vehicle - string indicating the vehicle type that is being designed
%  range   - design range [m]
%  payload - payload mass [kg]
%
% Outputs:
%  f - direct operating cost per flight [$]
%  c - nonlinear inequality constraints
%      - Available energy in the battery > energy required
%      - Motor power available > motor power required
%      - Takeoff mass > computed mass
%      - For a helicopter, kinetic energy in rotor enough to arrest 
%        autorotation descent rate


function [f,c] = computePerformance(x,vehicle,range,payload)

% Unpack design variables
rProp = x(1);
V = x(2);
mBattery = x(3);
mMotors = x(4);
mtow = x(5);

% Assumed values
batteryEnergyDensity = 230; % Expected pack energy density in 3-5 years [Wh/kg]
motorPowerDensity = 5; % kW/kg, including controller and other accessories in 3-5 years
dischargeDepthReserve = 0.95; % Can only use 95% of battery energy in reserve mission

% For the nominal mission compute the energy use, flight time, hover
% performance, and cruise performance
[ENominal,flightTime,hoverOutput,cruiseOutput] = simpleMission(vehicle,rProp,V,mtow*9.8,range);

% For the reserve mission compute energy use
[EReserve,~,~,~,~] = reserveMission(vehicle,rProp,V,mtow*9.8,range);

% Mass estimate
mass = configWeight(vehicle,rProp,mBattery,mMotors,mtow,hoverOutput,cruiseOutput,payload);

% Compute operating cost
C = operatingCost(vehicle,rProp,flightTime,ENominal,mass,cruiseOutput);

%% Objective is operating cost per flight
f = C.costPerFlight;

%% Constraints

% Constraint on available energy (E is in kW-hr)
c(1) = EReserve - mBattery * batteryEnergyDensity * dischargeDepthReserve / 1000; 

% Constraint on available motor power (kW)
c(2) = hoverOutput.PMax / 1000 - mMotors * motorPowerDensity;

% Constraint on MTOW
c(3) = mass.W - mtow * 9.8;

if strcmpi(vehicle,'helicopter')
    % Auto-rotation energy constraint => kinetic energy in blades has to be
    % twice that of vehicle in autorotation descent to be able to arrest
    % the descent.
    % See "Helicopter Theory" section 7-5, assume rotor CLmax is twice
    % hover CL. Rotor inertia is approximated as a solid rod. 
    c(4) = 0.5 * mass.m * hoverOutput.VAutoRotation^2 - 0.5 * 1/3 * mass.rotor * hoverOutput.Vtip^2;
end

end