% Estimates tooling cost based on part dimensions including material, rough
% machining, and finishing machining
%
% Inputs:
% length - part length [m]
% width  - part width [m]
% depth  - part depth [m]
%
% Output:
% cost - Tool cost [$]
%

function cost = toolingCost(length,width,depth)

% Material
toolSideOffset = 0.09; % Offset on each side of tool
toolDepthOffset = 0.03; % Offset at bottom of tool
toolCost = 10000; % Cost be m^3 of tooling material [$/m^3]

toolVolume = (length+2*toolSideOffset)*(width+2*toolSideOffset)*(depth+toolDepthOffset);
materialCost = toolCost*toolVolume; % Tooling material costs

% Machining (Rough Pass)
roughSFM = 200; % Roughing surface feet per minute
roughFPT = 0.003; % Roughing feed per tooth [in]
roughCostRate = 150/3600; % Cost to rough [$/s]
roughBitDiam = 0.05; % Rougher diameter [m]

roughBitDepth = roughBitDiam/4; % Rougher cut depth [m]
roughRPM = 3.82*roughSFM/(39.37*roughBitDiam); % Roughing RPM
roughFeed = roughFPT*roughRPM*2*0.00042; % Roughing Feed [m/s]
roughBitStep = 0.8*roughBitDiam; % Rougher step size

cutVolume = length*pi*depth*width/4; % Amount of material to rough out
roughTime = cutVolume / (roughFeed*roughBitStep*roughBitDepth); % Time for roughing
roughCost = roughTime*roughCostRate; % Roughing cost

% Machining (Finish Pass)
finishSFM = 400; % Roughing surface feet per minute
finishFPT = 0.004; % Roughing feed per tooth [in]
finishCostRate = 175/3600; % Cost to finish [$/s]
finishBitDiam = 0.006; % Finish diameter [m]
finishPasses = 5; % Number of surface passes

finishRPM = 3.82*finishSFM/(39.37*finishBitDiam); % Roughing RPM
finishFeed = finishFPT*finishRPM*2*0.00042; % Roughing Feed [m/s]

finishBitStep = 0.8*finishBitDiam; % Rougher step size
cutArea = length*ellipsePerimeter(width/2,depth)/2; % Amount of material to rough out
finishTime = cutArea / (finishFeed*finishBitStep) * finishPasses; % Time for roughing
finishCost = finishTime*finishCostRate; % Roughing cost

% Total Cost
cost = materialCost + roughCost + finishCost;


function p = ellipsePerimeter(a,b)
% Approximate solution to ellipse perimeter
h=(a-b)^2 / (a+b)^2;
p=pi*(a+b)*(1+3*h/(10+sqrt(4-3*h)));