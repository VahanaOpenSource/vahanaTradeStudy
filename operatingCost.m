% Estimate direct operating costs per flight including: acquisition cost,
% insurance cost, operating facility cost, energy cost, andn maintenance. 
%
% Inputs:
%  vehicle      - Vehicle type ('tiltwing' or 'helicopter')
%  rProp        - Propeller or rotor radius [m]
%  flightTime   - Flight time for nominal flight [sec]
%  E            - Energy use for flight [kW-hr]
%  mass         - Maximum takeoff mass [kg]
%  cruiseOutput - Structure containing cruise performance outputs
%
% Outputs:
%  C - Operating cost per flight [$]


function [C] = operatingCost(vehicle,rProp,flightTime,E,mass,cruiseOutput)

% Assumptions
C.flightHoursPerYear = 600;
C.flightsPerYear = C.flightHoursPerYear / (flightTime/3600);
C.vehicleLifeYears = 10;
C.nVehiclesPerFacility = 200; % Size of storage depot

% Tooling cost
C.toolCostPerVehicle = costBuildup(vehicle,rProp,cruiseOutput);

% Material cost
C.materialCostPerKg = 220; % Material plus assmebly cost
C.materialCost = C.materialCostPerKg * mass.structural;

% battery cost per kg
C.batteryCostPerKg = 161; % Roughly $700/kW-hr * 230 W-hr/kg
C.batteryCost = C.batteryCostPerKg * mass.battery;

% motor cost per kg
C.motorCostPerKg = 150; % Approx $1500 for 10 kg motor? + controller
C.motorCost = C.motorCostPerKg * mass.motors;

% servo cost 
if strcmpi(vehicle,'tiltwing')
    nServo = 14; % 8 props, 4 surfaces, 2 tilt
elseif strcmpi(vehicle,'helicopter')
    nServo = 8; % For cyclic (2x) / collective, tail rotor w/ redundancy
end
C.servoCost = nServo * 800; % Estimate $800 per servo in large quantities

% Avionics cost
C.avionicsCost = 30000; % guess for all sensors and computers in large quantities

% BRS Cost
if strcmpi(vehicle,'tiltwing')
    C.BRSCost = 5200;
elseif strcmpi(vehicle,'helicopter')
    C.BRSCost = 0;
end

% Total aquisition cost
C.acquisitionCost = C.batteryCost + C.motorCost + C.servoCost + C.avionicsCost + C.BRSCost + C.materialCost + C.toolCostPerVehicle; 
C.acquisitionCostPerFlight = C.acquisitionCost / (C.flightsPerYear * C.vehicleLifeYears);

% Insurance cost
% Follow R22 for estimate of 6.5% of acquisition cost
C.insuranceCostPerYear = C.acquisitionCost * 0.065;
C.insuranceCostPerFlight = C.insuranceCostPerYear / C.flightsPerYear;

% Facility rental cost
if strcmpi(vehicle,'tiltwing')
    C.vehicleFootprint = 1.2 * (8 * rProp + 1) * (4 * rProp + 3); % m^2, 20% for movement around aircraft for maintenance, etc.
elseif strcmpi(vehicle,'helicopter')
    C.vehicleFootprint = 1.2 * (2 * rProp)^2; % m^2, 20% for movement around aircraft for maintenance, etc.
end
C.areaCost = 10.7639 * 2 * 12; % $/m^2, $2/ft^2 per month assumed

% Facility cost = Vehicle footprint + 10x footprint for operations,
% averaged over # of vehicles at each facility
C.facilityCostPerYear = (C.vehicleFootprint + 10 * C.vehicleFootprint / C.nVehiclesPerFacility) * C.areaCost; 
C.facilityCostPerFlightHour = C.facilityCostPerYear / C.flightHoursPerYear;
C.facilityCostPerFlight = C.facilityCostPerFlightHour * flightTime / 3600;

% Electricity cost
% E * $/kWhr including 90% charging efficiency
% Average US electricity cost is $0.12 per kW-hr, up to $0.20 per kW-hr in CA
C.energyCostPerFlight = 0.12 * E / 0.9;

% Battery replacement cost
C.battLifeCycles = 2000;
C.batteryReplCostPerFlight = C.batteryCost / C.battLifeCycles; % 1 cycle per flight

% Motor replacement cost
C.motorLifeHrs = 6000;
C.motorReplCostPerFlight = flightTime / 3600 / C.motorLifeHrs * C.motorCost;

% Servo replacement cost
C.servoLifeHrs = 6000;
C.servoReplCostPerFlight = flightTime / 3600 / C.servoLifeHrs * C.servoCost;

% Maintenance cost
C.humanCostPerHour = 60;
if strcmpi(vehicle,'tiltwing')
    C.manHrPerFlightHour = 0.10; % periodic maintenance estimate
    C.manHrPerFlight = 0.2; % Inspection, battery swap estimate
elseif strcmpi(vehicle,'helicopter')
    C.manHrPerFlightHour = 0.05; % periodic maintenance estimate
    C.manHrPerFlight = 0.2; % Inspection, battery swap estimate
else
    error('Vehicle not recognized!');
end
C.laborCostPerFlight = (C.manHrPerFlightHour * flightTime / 3600 + C.manHrPerFlight) * C.humanCostPerHour;

% Cost per flight
C.costPerFlight = C.acquisitionCostPerFlight + C.insuranceCostPerFlight + C.facilityCostPerFlight + ...
    C.energyCostPerFlight + C.batteryReplCostPerFlight + C.motorReplCostPerFlight + ...
    C.servoReplCostPerFlight + C.laborCostPerFlight;

