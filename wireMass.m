% Estimate wire mass including DC power cables and communications cables
%
% Inputs:
% span           - wingspan [m]
% fuselageLength - fuselage length [m]
% fuselageHeight - fuselage height [m]
% power          - maximum DC power draw [W]
% xmotor         - vector of spanwise locations of each motor in fractions of wing semi-span
%
% Outputs:
% mass - estimated wire mass [kg]
%

function mass = wireMass(span,fuselageLength,fuselageHeight,power,xmotor)

nMotors = max(length(xmotor),1);

% Power Cables
P = power/nMotors;
cableDensity = 1e-5; % Approximate power cable pair density [kg/m/W], ~500 g/m for pair of wires carrying 50 kW

% Wires for each motor runs half fuselage length and height on average. Also runs out from center to location on wing.  
L = nMotors * fuselageLength / 2 + nMotors * fuselageHeight / 2 + sum(xmotor) * span/2;
massCables = cableDensity * P * L;

% Sensor wires
wireDensity = 0.0046; % kg/m
wiresPerBundle = 6; % Wires per bundle
L = L + 10 * fuselageLength + 4 * span; % Additional wires for motor controllers, airdata, lights, servos, sensors
massWires = 2 * wireDensity * wiresPerBundle * L; % Sensor wires for motors

mass = massCables+massWires;