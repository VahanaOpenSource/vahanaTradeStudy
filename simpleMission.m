% Estimate time and energy use for a simple mission VTOL mission
%
% Inputs:
%   vehicle - vehicle type ('tiltwing' or 'helicopter')
%   rProp   - prop/rotor radius [m]
%   V       - cruise speed [m/s]
%   W       - weight [N]
%   range   - range for each segment of the mission [m]
%
% Outputs:
%   E            - Total energy use in nominal mission [kW-hr]
%   t            - Flight time for nominal mission [sec]
%   hoverOutput  - Structure with computed hover performance
%   cruiseOutput - Structure with computed cruise performance
%

function [E,t,hoverOutput,cruiseOutput] = simpleMission(vehicle,rProp,V,W,range)

% Basic mission
hoverTime = 180*numel(range); % sec to account for VTOL takeoff and climb, transition, transition, VTOL descent and landing

% Compute cruise time
cruiseTime = sum(range) / V; % sec

% Compute cruise performance
cruiseOutput = cruisePower(vehicle,rProp,V,W);

% Compute hover performance
hoverOutput = hoverPower(vehicle,rProp,W,cruiseOutput);

% Compute total energy use (kW-hr)
E = (hoverOutput.PBattery * hoverTime + cruiseOutput.PBattery * cruiseTime) * 2.77778e-7; % kW-hr

% Compute total flight time
t = hoverTime + cruiseTime;

end