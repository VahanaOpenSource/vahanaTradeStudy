% Estimate fuselage structural mass assuming a structural keel to take
% bending and torsional loads.
%
% Inputs:
% length - fuselage length [m]
% width  - fuselage width [m]
% height - fuselage height [m]
% span   - wingspan [m]
% weight - vehicle weight [N]
%
% Outputs:
% mass - Mass of the fuselage [kg]


function mass = fuselageMass(length,width,height,span,weight)

ng = 3.8; % Max g lift
nl = 3.5; % Landing load factor
sf = 1.5; % Safety factor

% Load material properties
materials;

% Skin areal weight
arealWeight = bid.minThk*bid.rho+core.minThk*core.rho+paint.thk*paint.rho;

% Skin Mass - approximate area of ellipsoid given length, width, height,
% (see https://en.wikipedia.org/wiki/Ellipsoid#Surface_area)
Swet = 4*pi*(((length*width/4)^1.6 + (length*height/4)^1.6 + (width*height/4)^1.6)/3)^(1/1.6);
skinMass = Swet*arealWeight;

% Bulkhead Mass
bulkheadMass = 3*pi*height*width/4*arealWeight;

% Canopy Mass
canopyMass = Swet/8*canopy.thk*canopy.rho;

% Keel Mass due to lift
L = ng*weight*sf; % Lift
M = L*length/2; % Peak moment
beamWidth = width/3; % Keel width
beamHeight = height/10; % Keel height

A = M*beamHeight/(4*uni.stress*(beamHeight/2)^2);
massKeel = A*length*uni.rho;

% Keel Mass due to torsion
M = 0.25*L*span/2; % Wing torsion
A = beamHeight*beamWidth;
t = 0.5*M/(bid.shear*A);
massKeel = massKeel+2*(beamHeight+beamWidth)*t*bid.rho;

% Keel Mass due to landing
F = sf*weight*nl*sqrt(sum([1 0.8].^2))/2; % Landing force, side landing
A = F/steel.shear; % Requried bolt area
d = 2*sqrt(A/pi); % Bolt diameter
t = F/(d*bid.bearing); % Laminate thickness
V = pi*(20*t)^2*t/3; % Pad up volume
massKeel = massKeel + 4*V*bid.rho; % Mass of all 4 pad ups

% Total mass
mass = skinMass+bulkheadMass+canopyMass+massKeel;