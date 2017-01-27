% Estimate tooling cost of a vehicle by summing tooling costs for all major
% components
%
% Inputs:
%  vehicle      - Vehicle type ('tiltwing' or 'helicopter')
%  rProp        - Prop/rotor radius [m]
%  cruiseOutput - Structure with cruise performance data
%
% Outputs:
%  toolCostPerVehicle - Estimate of tool cost per vehicle [$]
%

function toolCostPerVehicle = costBuildup(vehicle,rProp,cruiseOutput)

% Inputs
fuselageWidth = 1;
fuselageLength = 5;
toc = 0.15;  % Wing / canard thickness
propRadius = rProp;
propChord = 0.15*propRadius; % Max chord
xhinge = 0.8;
winglet = 0.2;

partsPerTool = 1000;

if strcmpi(vehicle,'tiltwing')
    
    fuselageHeight = 1.3; % Guess
    span = cruiseOutput.bRef;
    chord = cruiseOutput.cRef;

    % Wing Tooling
    wingToolCost(1) = toolingCost((span-fuselageWidth)/2,toc*chord,chord*.2); % Leading edge
    wingToolCost(2) = toolingCost((span-fuselageWidth)/2,toc*chord*0.7,chord*.2); % Aft spar
    wingToolCost(3) = toolingCost((span-fuselageWidth)/2,chord*0.75,0.02)*2; % Upper/Lower skin
    wingToolCost(4) = toolingCost(span,toc*chord,chord*.20); % Forward spar
    wingToolCost = 2*(2*sum(wingToolCost(1:3))+wingToolCost(4)); % Total wing tooling cost (matched tooling)
    
    % Winglet Tooling
    wingletToolCost(1) = toolingCost(winglet*span/2,toc*chord,chord*.2); % Leading edge
    wingletToolCost(2) = toolingCost(winglet*span/2,toc*chord*0.7,chord*.2); % Aft spar
    wingletToolCost(3) = toolingCost(winglet*span/2,chord*0.75,0.02)*2; % Upper/Lower skin
    wingletToolCost(4) = toolingCost(winglet*span/2,toc*chord,chord*.20); % Forward spar
    wingletToolCost = 4*sum(wingletToolCost); % Total winglet tooling cost (matched tooling)
    
    % Canard Tooling
    canardToolCost = wingToolCost; % Total canard tooling cost
    
    % Fuselage Tooling
    fuselageToolCost(1) = toolingCost(fuselageLength*.8,fuselageHeight,fuselageWidth/2)*2; % Right/Left skin
    fuselageToolCost(2) = toolingCost(fuselageLength*.8,fuselageWidth/2,fuselageHeight/4); % Keel
    fuselageToolCost(3) = toolingCost(fuselageWidth,fuselageHeight,0.02)*2; % Fwd/Aft Bulkhead
    fuselageToolCost(4) = toolingCost(fuselageLength*.1,fuselageWidth,fuselageHeight/3); % Nose/Tail cone
    fuselageToolCost = 2*sum(fuselageToolCost); % Total fuselage tooling cost (matched tooling)
    
    % Prop Tooling
    propToolCost(1) = toolingCost(propRadius,propChord,propChord*toc/2)*2; % Skin
    propToolCost(2) = toolingCost(propRadius,propChord*toc,propChord/4)*2; % Spar tool
    propToolCost = 4*sum(propToolCost); % Total propeller tooling cost (left/right hand) (matched tooling)
    
    % Control Surface (2 tools)
    controlToolCost(1) = toolingCost((span-fuselageWidth)/2,(1-xhinge)*chord,chord*toc/4); % Skin
    controlToolCost(2) = toolingCost((span-fuselageWidth)/2,(1-xhinge)*chord,chord*toc/4); % Skin
    controlToolCost = 8*sum(controlToolCost); % Total wing tooling (matched tooling)
    
    % Total tool cost
    totalToolCost = wingToolCost+canardToolCost+fuselageToolCost+propToolCost+controlToolCost+wingletToolCost;
    
elseif strcmpi(vehicle,'helicopter')
    fuselageHeight = 1.6; % Guess

    % Fuselage Tooling
    fuselageToolCost(1) = toolingCost(fuselageLength*.8,fuselageHeight,fuselageWidth/2)*2; % Right/Left skin
    fuselageToolCost(2) = toolingCost(fuselageLength*.8,fuselageWidth/2,fuselageHeight/4); % Keel
    fuselageToolCost(3) = toolingCost(fuselageWidth,fuselageHeight,0.02)*2; % Fwd/Aft Bulkhead
    fuselageToolCost(4) = toolingCost(fuselageLength*.1,fuselageWidth,fuselageHeight/3); % Nose/Tail cone
    fuselageToolCost = 2*sum(fuselageToolCost); % Total fuselage tooling cost (matched tooling)
    
    % Rotor Tooling
    rotorToolCost(1) = toolingCost(propRadius,propChord,propChord*toc/2)*2; % Skin
    rotorToolCost(2) = toolingCost(propRadius,propChord*toc,propChord/4)*2; % Spar tool
    rotorToolCost = 2*sum(rotorToolCost); % Total rotor tooling cost (matched tooling)
    
    % Tail Rotor Tooling
    tailRotorToolCost(1) = toolingCost(propRadius/4,propChord/4,propChord/4*toc/2)*2; % Skin
    tailRotorToolCost(2) = toolingCost(propRadius/4,propChord/4*toc,propChord/4/4)*2; % Spar tool
    tailRotorToolCost = 2*sum(tailRotorToolCost); % Total rotor tooling cost (matched tooling)
    
    % Total tool cost
    totalToolCost = fuselageToolCost+rotorToolCost+tailRotorToolCost;
    
else
    error('vehicle not recognized!');
end

toolCostPerVehicle = totalToolCost/partsPerTool;
