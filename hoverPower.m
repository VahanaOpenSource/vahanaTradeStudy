% Estimate hover performance
%
% Inputs:
%  vehicle      - Vehicle type ('tiltwing' or 'helicopter')
%  rProp        - Prop/rotor radius
%  W            - Weight
%  cruiseOutput - Cruise data
%
% Outputs:
%  hoverOutput - Structure with hover performance values
%


function [hoverOutput] = hoverPower(vehicle,rProp,W,cruiseOutput)

% Altitude, compute atmospheric properties
rho = 1.225;

% Blade parameters
hoverOutput.Cd0 = 0.012; % Blade airfoil profile drag coefficient
hoverOutput.sigma = 0.1; % Solidity (could estimate from Ct assuming some average blade CL)

% Different assumptions per vehicle
if strcmpi(vehicle,'tiltwing')
    
    hoverOutput.nProp = 8; % Number of props / motors
    hoverOutput.ToverW = 1.7; % Max required T/W to handle rotor out w/ manuever margin
    hoverOutput.k = 1.15; % Effective disk area factor (see "Helicopter Theory" Section 2-6.2)
    hoverOutput.etaMotor = 0.85; % Assumed electric motor efficiency
    
    % Tip Mach number constraint for noise reasons at max thrust condition
    hoverOutput.MTip = 0.65;
    
    % Tip speed limit
    hoverOutput.Vtip = 340.2940 * hoverOutput.MTip / sqrt(hoverOutput.ToverW); % Limit tip speed at max thrust, not hover
    hoverOutput.omega = hoverOutput.Vtip / rProp;
    
    % Thrust per prop / rotor at hover
    hoverOutput.THover = W / hoverOutput.nProp;
    
    % Compute thrust coefficient
    hoverOutput.Ct = hoverOutput.THover / (rho * pi * rProp^2 * hoverOutput.Vtip^2);
    
    % Average blade CL (see "Helicopter Theory" section 2-6.3)
    hoverOutput.AvgCL = 6 * hoverOutput.Ct / hoverOutput.sigma;
    
    % Hover Power
    hoverOutput.PHover = hoverOutput.nProp * hoverOutput.THover * ...
        (hoverOutput.k .* sqrt(hoverOutput.THover ./ (2 * rho * pi * rProp.^2)) + ...
        hoverOutput.sigma * hoverOutput.Cd0 / 8 * (hoverOutput.Vtip)^3 ./ (hoverOutput.THover ./ (rho * pi * rProp.^2)));
    hoverOutput.FOM = hoverOutput.nProp * hoverOutput.THover * sqrt(hoverOutput.THover ./ (2 * rho * pi * rProp.^2)) / hoverOutput.PHover;
    
    % Battery power
    hoverOutput.PBattery = hoverOutput.PHover / hoverOutput.etaMotor;
    
    % Maximum thrust per motor
    hoverOutput.TMax = hoverOutput.THover * hoverOutput.ToverW;
    
    % Maximum shaft power required (for motor sizing)
    % Note: Tilt-wing multirotor increases thrust by increasing RPM at constant collective
    hoverOutput.PMax = hoverOutput.nProp * hoverOutput.TMax * ...
        (hoverOutput.k .* sqrt(hoverOutput.TMax ./ (2 * rho * pi * rProp.^2)) + ...
        hoverOutput.sigma * hoverOutput.Cd0 / 8 * (hoverOutput.Vtip * sqrt(hoverOutput.ToverW))^3 ./ (hoverOutput.TMax ./ (rho * pi * rProp.^2)));
    
    % Max battery power
    hoverOutput.PMaxBattery = hoverOutput.PMax / hoverOutput.etaMotor;
    
    % Maximum torque per motor
    hoverOutput.QMax = hoverOutput.PMax / (hoverOutput.omega * sqrt(hoverOutput.ToverW));

elseif strcmpi(vehicle,'helicopter')
    
    hoverOutput.nProp = 1; % Number of rotors
    hoverOutput.ToverW = 1.1; % Max required T/W for climb and operating at higher altitudes
    hoverOutput.k = 1.15; % Effective disk area factor (see "Helicopter Theory" Section 2-6.2)
    hoverOutput.etaMotor = 0.85 * 0.98; % Assumed motor and gearbox efficiencies (85% and 98% respectively)
    
    hoverOutput.omega = cruiseOutput.omega;
    hoverOutput.Vtip = hoverOutput.omega * rProp;
    
    % Thrust per prop / rotor at hover
    hoverOutput.THover = W / hoverOutput.nProp;
    
    % Compute thrust coefficient
    hoverOutput.Ct = hoverOutput.THover / (rho * pi * rProp^2 * hoverOutput.Vtip^2);
    
    % Average blade CL (see "Helicopter Theory" Section 2-6.4)
    hoverOutput.AvgCL = 6 * hoverOutput.Ct / hoverOutput.sigma;
    
    % Auto-rotation descent rate (see "Helicopter Theory" Section 3-2)
    hoverOutput.VAutoRotation = 1.16 * sqrt(hoverOutput.THover / (pi * rProp^2));
    
    % Hover Power
    hoverOutput.PHover = hoverOutput.nProp * hoverOutput.THover * ...
        (hoverOutput.k .* sqrt(hoverOutput.THover ./ (2 * rho * pi * rProp.^2)) + ...
        hoverOutput.sigma * hoverOutput.Cd0 / 8 * (hoverOutput.Vtip)^3 ./ (hoverOutput.THover ./ (rho * pi * rProp.^2)));
    hoverOutput.FOM = hoverOutput.nProp * hoverOutput.THover * sqrt(hoverOutput.THover ./ (2 * rho * pi * rProp.^2)) / hoverOutput.PHover;
    
    % Battery power
    % ~10% power to tail rotor (see "Princples of Helicopter Aerodynamics" by Leishman)
    hoverOutput.PTailRotor = 0.1 * hoverOutput.PHover;
    hoverOutput.PBattery = (hoverOutput.PHover + hoverOutput.PTailRotor) / hoverOutput.etaMotor;
    
    % Maximum thrust per motor
    hoverOutput.TMax = hoverOutput.THover * hoverOutput.ToverW;
    
    % Maximum shaft power required (for motor sizing)
    % Note: Helicopter increases thrust by increasing collective with constant RPM
    hoverOutput.PMax = hoverOutput.nProp * hoverOutput.TMax * ...
        (hoverOutput.k .* sqrt(hoverOutput.TMax ./ (2 * rho * pi * rProp.^2)) + ...
        hoverOutput.sigma * hoverOutput.Cd0 / 8 * (hoverOutput.Vtip)^3 ./ (hoverOutput.TMax ./ (rho * pi * rProp.^2)));
        
    % ~15% power to tail rotor for sizing (see "Princples of Helicopter Aerodynamics" by Leishman)
    hoverOutput.PMax = 1.15 * hoverOutput.PMax;
    
    % Max battery power
    hoverOutput.PMaxBattery = hoverOutput.PMax / hoverOutput.etaMotor;
    
    % Maximum torque per motor
    hoverOutput.QMax = hoverOutput.PMax / hoverOutput.omega;
    
else
    
    error('Unrecognized vehicle!')

end



