% Estimate cruise performance
%
% Inputs:
%  vehicle - vehicle type ('tiltwing' or 'helicopter')
%  rProp   - prop/rotor radius [m]
%  V       - cruise speed [m/s]
%  W       - weight [N]
%
% Outputs:
%  cruiseOutputs - structure with cruise performance values
%

function [cruiseOutput] = cruisePower(vehicle,rProp,V,W)

% Altitude, compute atmospheric properties
rho = 1.225;

% Fuselage / landing gear drag area
cruiseOutput.SCdFuse = 0.35;

if strcmpi(vehicle,'tiltwing')
    
    % Specify stall conditions
    VStall = 35; % m/s
    CLmax = 1.1; % Whole aircraft CL, section Clmax much higher
    
    % Compute Wingspan assuming 2 props per wing with outboard props are at
    % wingtips, 1 meter wide fuselage plus clearance between props and fuselage
    cruiseOutput.bRef = 6 * rProp + 1.2; % Rough distance between hubs of outermost props
    
    % Compute reference area (counting both wings)
    cruiseOutput.SRef = W / (0.5 * rho * VStall^2 * CLmax);

    % Compute reference chord (chord of each wing)
    cruiseOutput.cRef = 0.5 * cruiseOutput.SRef / cruiseOutput.bRef; 
        
    % Equivalent aspect ratio
    cruiseOutput.AR = cruiseOutput.bRef^2 / cruiseOutput.SRef;
    
    % Motor efficiency
    cruiseOutput.etaMotor = 0.85;

    % Wing profile drag coefficent
    cruiseOutput.Cd0Wing = 0.012;
    
    % Overall profile drag
    cruiseOutput.Cd0 = cruiseOutput.Cd0Wing + cruiseOutput.SCdFuse / cruiseOutput.SRef;
    
    % Span efficiency
    cruiseOutput.e = 1.3;
    
    % Solve for CL at cruise
    cruiseOutput.CL = W / (0.5 * rho * V^2 * cruiseOutput.SRef);
    
    % Prop efficiency
    cruiseOutput.etaProp = 0.8;

    % Estimate drag at cruise using quadratic drag polar
    cruiseOutput.D = 0.5 * rho * V^2 * (cruiseOutput.SRef * (cruiseOutput.Cd0 + ...
        cruiseOutput.CL^2 / (pi * cruiseOutput.AR * cruiseOutput.e)));
    
    % Compute cruise power estimate
    cruiseOutput.PCruise = cruiseOutput.D * V;
    
    % Battery power
    cruiseOutput.PBattery = cruiseOutput.PCruise / cruiseOutput.etaProp / cruiseOutput.etaMotor;
    
    % Cruise L/D
    cruiseOutput.LoverD = W / cruiseOutput.D;
    
elseif strcmpi(vehicle,'helicopter')
    
    % Motor efficiency
    cruiseOutput.etaMotor = 0.85 * 0.98; % Assumed motor and gearbox efficiencies (85%, and 98% respectively)
    
    % Tip Mach number constraint
    cruiseOutput.MTip = 0.65;
    
    % Tip loss factor 
    cruiseOutput.B = 0.97;
    
    % Blade solidity
    cruiseOutput.sigma = 0.1;
    
    % Blade profile drag coefficient
    cruiseOutput.Cd0 = 0.012;
    
    % Compute rotation rate at cruise to be at tip mach limit
    cruiseOutput.omega = (340.2940 * cruiseOutput.MTip - V) / rProp;
    
    % Fuselage drag
    cruiseOutput.D = 0.5 * rho * V^2 * cruiseOutput.SCdFuse;
    
    % Inflow angle 
    cruiseOutput.alpha = atan(cruiseOutput.D/W);
    
    % Compute advance ratio
    cruiseOutput.mu = V * cos(cruiseOutput.alpha) / (cruiseOutput.omega * rProp);
        
    % Thrust coefficient (including tip loss factor for effective disk area)
    cruiseOutput.Ct = W / (rho * pi * rProp^2 * cruiseOutput.B^2 * cruiseOutput.omega^2 * rProp^2);
    
    % Solve for induced velocity /w Newton method (see "Helicopter Theory" section 4.1.1)
    cruiseOutput.lambda = cruiseOutput.mu * tan(cruiseOutput.alpha) + cruiseOutput.Ct / ...
        (2 * sqrt(cruiseOutput.mu^2 + cruiseOutput.Ct/2));
    for i = 1:5
        cruiseOutput.lambda  = (cruiseOutput.mu * tan(cruiseOutput.alpha) + ...
            cruiseOutput.Ct / 2 * (cruiseOutput.mu^2 + 2*cruiseOutput.lambda^2) / ...
            (cruiseOutput.mu^2 + cruiseOutput.lambda^2)^1.5) / ...
            (1 + cruiseOutput.Ct/2 * cruiseOutput.lambda / (cruiseOutput.mu^2 + cruiseOutput.lambda^2)^1.5);
    end
    cruiseOutput.v = cruiseOutput.lambda * cruiseOutput.omega * rProp - V * sin(cruiseOutput.alpha);
    
    % Power in forward flight (see "Helicopter Theory" section 5-12)
    cruiseOutput.PCruise = W * (V * sin(cruiseOutput.alpha) + 1.3 * cosh(8 * cruiseOutput.mu^2) * cruiseOutput.v + ...
        cruiseOutput.Cd0 * cruiseOutput.omega * rProp * ...
        (1 + 4.5 * cruiseOutput.mu^2 + 1.61 * cruiseOutput.mu^3.7) * ...
        (1 - (0.03 + 0.1 * cruiseOutput.mu + 0.05 * sin(4.304 * cruiseOutput.mu - 0.20)) * ...
        (1-cos(cruiseOutput.alpha)^2)) / 8 / (cruiseOutput.Ct / cruiseOutput.sigma));
    
    % 10% power added for helicopter tail rotor
    cruiseOutput.PCruise = 1.1 * cruiseOutput.PCruise;
    
    % Equivalent L/D, assuming power = D * V and L = W
    cruiseOutput.LoverD = W / (cruiseOutput.PCruise / V);
    
    % Battery power
    cruiseOutput.PBattery = cruiseOutput.PCruise / cruiseOutput.etaMotor;

else
    
    error('Unrecognized vehicle!')
    
end