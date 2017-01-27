% Estimate time and energy use for a reserve VTOL mission
%
% Inputs:
%   vehicle - vehicle type
%   rProp   - prop/rotor radius
%   V       - cruise speed
%   W       - weight
%
% Outputs:
%   E            - Total energy use in reserve mission
%   t            - Flight time for reserve mission
%   hoverOutput  - Computed hover performance
%   cruiseOutput - Computed cruise performance
%   loiterOutput - Computed loiter performance
%

function [E,t,hoverOutput,cruiseOutput,loiterOutput] = reserveMission(vehicle,rProp,V,W,range)

% Reserve mission
hoverTime = 180 * (numel(range)+1); % sec to account for VTOL takeoff and climb, transition, transition, VTOL descent and landing and repeated for diversion

% Compute cruise time
cruiseTime = sum(range) / V; % sec

% Loiter time
loiterTime = 17 * 60; % 20 minute total reserve

% Compute cruise performance
cruiseOutput = cruisePower(vehicle,rProp,V,W);

% Compute hover performance
hoverOutput = hoverPower(vehicle,rProp,W,cruiseOutput);

% Compute loiter performance
loiterOutput = loiterPower(vehicle,rProp,V,W,cruiseOutput);

% Compute total energy use (kW-hr)
E = (hoverOutput.PBattery * hoverTime + cruiseOutput.PBattery * cruiseTime + loiterOutput.PBattery * loiterTime) * 2.77778e-7; % kW-hr

% Compute total flight time
t = hoverTime + cruiseTime + loiterTime;

end