% Estimate loiter performance
%
% Inputs:
%  vehicle      - Vehicle type ('tiltwing' or 'helicopter')
%  rProp        - Prop/rotor radius [m]
%  W            - Weight [N]
%  cruiseOutput - Output of cruise analysis
%  hoverOutput  - Output of hover analysis
%
% Outputs:
%  loiterOutputs - structure with loiter performance values


function [loiterOutput] = loiterPower(vehicle,rProp,V,W,cruiseOutput)

% Altitude, compute atmospheric properties
rho = 1.225;

loiterOutput.PBattery = cruiseOutput.PBattery;

if strcmpi(vehicle,'tiltwing')
    
    % Note typical loiter at CL^1.5/CD for quadratic drag model likely is
    % beyond CLMax for the airplane
    % Lift coefficent at loiter a little below vehicle CLmax of ~1.1
    loiterOutput.CL = 1.0;
    
    % Compute velocity
    loiterOutput.VLoiter = sqrt(2 * W / (rho * cruiseOutput.SRef * loiterOutput.CL));
    
    % Compute drag
    loiterOutput.D = 0.5 * rho * loiterOutput.VLoiter^2 * (cruiseOutput.SRef * (cruiseOutput.Cd0 + ...
        loiterOutput.CL^2 / (pi * cruiseOutput.AR * cruiseOutput.e)) + cruiseOutput.SCdFuse);
    
    % Compute cruise power estimate
    loiterOutput.PCruise = loiterOutput.D * loiterOutput.VLoiter;
    
    % Battery power
    loiterOutput.PBattery = loiterOutput.PCruise / cruiseOutput.etaProp / cruiseOutput.etaMotor;
    
    % Cruise L/D
    loiterOutput.LoverD = W / loiterOutput.D;
    
elseif strcmpi(vehicle,'helicopter')
    
    % Thrust coefficient (including tip loss factor for effective disk area
    Ct = W / (rho * pi * rProp^2 * cruiseOutput.B^2 * cruiseOutput.omega^2 * rProp^2);
    
    % Find velocity for min power
    loiterOutput.V = fminbnd(@(VLoiter) loiterPower(VLoiter),0,V);

    % Get loiter power
    loiterOutput.PLoiter = loiterPower(loiterOutput.V);
    
    % Battery Power
    loiterOutput.PBattery = loiterOutput.PLoiter / cruiseOutput.etaMotor;
    
else
    error('Unrecognized vehicle!')
end


    %% Compute helicopter power in at a given speed
    function PLoiter = loiterPower(VLoiter)
        
        % Fuselage drag
        D = 0.5 * rho * VLoiter^2 * cruiseOutput.SCdFuse;
        
        % Inflow angle
        alpha = atan(D/W);
        
        % Compute advance ratio
        mu = VLoiter * cos(alpha) / (cruiseOutput.omega * rProp);
        
        % Solve for induced velocity /w Newton method (see "Helicopter Theory" section 4.1.1)
        lambda = mu * tan(alpha) + Ct / ...
            (2 * sqrt(mu^2 + Ct/2));
        for i = 1:5
            lambda  = (mu * tan(alpha) + ...
                Ct / 2 * (mu^2 + 2*lambda^2) / ...
                (mu^2 + lambda^2)^1.5) / ...
                (1 + Ct/2 * lambda / (mu^2 + lambda^2)^1.5);
        end
        v = lambda * cruiseOutput.omega * rProp - VLoiter * sin(alpha);
        
        % Power in forward flight (see "Helicopter Theory" section 5-12)
        PLoiter = W * (VLoiter * sin(alpha) + 1.3 * cosh(8 * mu^2) * v + ...
            cruiseOutput.Cd0 * cruiseOutput.omega * rProp * ...
            (1 + 4.5 * mu^2 + 1.61 * mu^3.7) * ...
            (1 - (0.03 + 0.1 * mu + 0.05 * sin(4.304 * mu - 0.20)) * ...
            (1-cos(alpha)^2)) / 8 / (Ct / cruiseOutput.sigma));
    end

end
