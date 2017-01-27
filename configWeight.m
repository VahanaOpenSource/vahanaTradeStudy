% Weight models for both configurations
%
% Inputs:
%  vehicle      - vehicle type ('tiltwing' or 'helicopter')
%  rProp        - prop/rotor radius [m]
%  mBattery     - battery mass [kg]
%  mMotors      - total motor mass [kg]
%  mtow         - maximum takeoff mass [kg]
%  hoverOutput  - Structure with computed hover performance
%  cruiseOutput - Structure with computed cruise performance
%  payload      - Payload mass [kg]
%
% Outputs:
%  mass - structure with component masses [kg] and maximum takeoff weight [N]


function [mass] = configWeight(vehicle,rProp,mBattery,mMotors,mtow,hoverOutput,cruiseOutput,payload)

% Total payload mass
mass.payload = payload;

% Fixed weight components between configs
mass.seat = 15; % kg
mass.avionics = 15; % kg, Flight computer, sensors, BMS, backup power battery

% Motor and battery weight (design variables)
mass.motors = mMotors;
mass.battery = mBattery;

mPerServo = 0.65; % per servo in class needed
mPerTilt = 4; % per wing tilt actuator (prelim design)

if strcmpi(vehicle,'tiltwing')
    
    % Servo weight
    mass.servos = mPerServo * 12; % 4 ctrl surfaces + variable pitch actuators
    
    % Tilt mechanism weight
    mass.tilt = 2 * mPerTilt; % 2 wing tilt mechanisms

    % Ballistic Recovery System 
    % For 1600 lbf experimental aircraft, should actually scale with size
    mass.brs = 16;

    % Wing weight estimate (taking 40% of load w/ 20% semi-span winglets)
    % Inboard motor tips located 0.5 m from centerline, with 0.05 m gap to
    % outer motors
    mass.wing = wingMass(mtow*9.8,cruiseOutput.bRef,cruiseOutput.cRef,0.2,0.4,...
        [2*(0.5 + rProp) / cruiseOutput.bRef,2*(0.5 + 3*rProp + 0.05)/cruiseOutput.bRef], hoverOutput.TMax);
    
    % Canard weight estimate (taking 60% of load)
    % Inboard motor tips located 0.5 m from centerline, with 0.05 m gap to
    % outer motors
    mass.canard = wingMass(mtow*9.8,cruiseOutput.bRef,cruiseOutput.cRef,0,0.6,...
        [2*(0.5 + rProp) / cruiseOutput.bRef,2*(0.5 + 3*rProp + 0.05)/cruiseOutput.bRef], hoverOutput.TMax);
    
    % Propeller blade mass plus 2 kg per VP hub
    mass.props = 8 * propMass(rProp,hoverOutput.TMax);
    mass.hub = 8 * 2;
    
    % Fuselage mass assuming 5 m long fuselage, 1 m wide fuselage, and 1.65
    % m tall fuselage
    mass.fuselage = fuselageMass(5,1,1.65,cruiseOutput.bRef,mtow*9.8);
    
    % Landing gear mass is assumed to be 2% of MTOW for helicopters
    mass.lg = 0.02 * mtow;
    
    % Wire mass estimates
    % Inboard motor tips located 0.5 m from centerline, with 0.05 m gap to
    % outer motors
    mass.wire = wireMass(cruiseOutput.bRef,5,1.65,hoverOutput.PMaxBattery,[2*(0.5 + rProp) / cruiseOutput.bRef * ones(1,4),2*(0.5 + 3*rProp + 0.05)/cruiseOutput.bRef * ones(1,4)]);

    % Total structural mass (material cost)
    mass.structural = mass.wing + mass.canard + mass.props + mass.hub + mass.fuselage + mass.lg;
    
    % Total mass + 10% Fudge factor
    mass.m = 1.1 * (mass.payload + mass.seat + mass.avionics + mass.servos + ...
        mass.tilt + mass.structural +  ...
        mass.battery + mass.motors + mass.wire + mass.brs);
        
elseif strcmpi(vehicle,'helicopter')
    
    % Servo weight
    mass.servos = mPerServo * 8; % 8 for redundant collective, cyclic (2x), tail rotor
   
    % No BRS for helicopter
    mass.brs = 0;

    % Rotor mass plus assumed 4% of mtow for hub and linkages
    mass.rotor = propMass(rProp,hoverOutput.TMax);
    mass.hub = 0.04 * mtow;
    
    % Tail rotor mass (20% main rotor radius), assuming moment arm of 1.25x
    % rotor radius, need to be capable of providing 1.5x thrust required to
    % fight max rotor torque
    mass.tailRotor = propMass(rProp/5,1.5*hoverOutput.QMax/(1.25 * rProp)); 
    
    % Transmission mass
    % Estimate from OH-58 gearbox study that has a lower bound of 0.26 lb/Hp
    % https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19880020744.pdf    
    transmissionPowerDensity = 6.3; % kW/kg
    mass.transmission = hoverOutput.PMax / 1000 / transmissionPowerDensity;
    
    % Fuselage mass assuming fuselage length of 1.5 m nose plus 1.25x rotor
    % radius for tailboom length, 1 meter wide and 2 meter tall fuselage.
    mass.fuselage = fuselageMass(1.5+1.25*rProp,1,2,1,mtow*9.8);
    
    % Landing gear mass is assumed to be 2% of MTOW for helicopters
    mass.lg = 0.02 * mtow;
    
    % Wire mass assuming motors located close to battery
    mass.wire = wireMass(0,1.5+1.25*rProp,2,hoverOutput.PMaxBattery,0);
    
    % Total structural mass (material cost)
    mass.structural = mass.rotor + mass.hub + mass.tailRotor + mass.fuselage + mass.lg;
    
    % Total mass + 10% Fudge factor
    mass.m = 1.1 * (mass.payload + mass.seat + mass.avionics + mass.servos + ...
        mass.transmission + mass.structural + ...
        mass.battery + mass.motors + mass.wire + mass.brs);
    
else
    error('Unrecognized vehicle!') 
end

mass.W = mass.m * 9.8;

