% Estimate propeller blade mass 
%
% Inputs:
% R - rotor radius [m]
% T - maximum thrust [N]
%
% Outputs:
% mass - Mass of the blades for one propeller [kg]
%

function mass = propMass(R,T)

%% Setup
chord = 0.1 * R; % Assumed prop chord 
nBlades = 3; % Number of blades
N = 5; % Number of radial points
sf = 1.5; % Safety factor
toc = 0.12; % Average blade t/c
fwdWeb = [0.25 0.35]; %Forward web location x/c
xShear = 0.25; %Approximate shear center
rootLength = R / 10; %Root fitting length [m]
fudge = 1.2; % Fudge factor to account for misc items
sound = 340.2940; % Speed of sound [m/s]
tipMach = 0.65; % Tip mach number
cmocl = 0.02 / 1; % Ratio of cm/cl for sizing torsion (magnitude)

%% Load material properties
materials;

%% Airfoil
naca = 5*toc*[0.2969 -0.1260 -0.3516 0.2843 -0.1015]'; % Thickness distribution for NACA 4-series airfoil
coord = unique([fwdWeb linspace(0,1,N)])';
coord(:,2) = [coord(:,1).^0.5 coord(:,1) coord(:,1).^2 coord(:,1).^3 coord(:,1).^4]*naca;
coord = [flipud(coord(2:end,:)); coord*[1 0;0 -1]];
coord(:,1) = coord(:,1)-xShear;

%% Beam Geometry
x = linspace(0,R,N);
dx = x(2)-x(1);
fwdWeb = fwdWeb - xShear(1);

%% Loads
omega = sound*tipMach/R; % Rotational speed (for CF calc)
F = sf*3*T/R^3*x.^2/nBlades; % Force distribution
Q = F*chord*cmocl; % Torque distribution

% Initial mass estimates
box = coord*chord; % OML coordinates
L = sum(sqrt(sum(diff(box).^2,2))); % Skin length
y0 = (max(box(:,2))-min(box(:,2)))/2; % Max thickness
M0 = sf*T/nBlades*0.75*R; % Bending moment
m = uni.rho*dx*M0/(2*uni.stress*y0)+L*bid.minThk*dx*bid.rho; % Assumed mass distribution
m = m*ones(1,N);
error = 1; % Initialize error
tolerance = 1e-8; % Mass tolerance
massOld = sum(m);

%% General structural properties
% Torsion Properties
Ae = polyarea(box(:,1),box(:,2)); % Enclosed wing area
skinLength = sum(sqrt(sum(diff(box).^2,2)));

% Flap Properties
box = coord; % Get airfoil coordinates
box(box(:,1)>fwdWeb(2),:) = [];
box(box(:,1)<fwdWeb(1),:) = []; % Trim coordinates
seg{1} = box(box(:,2)>mean(box(:,2)),:)*chord; % Upper fwd segment
seg{2} = box(box(:,2)<mean(box(:,2)),:)*chord; % Lower fwd segment

% Flap/drag inertia
capInertia=0; capLength=0;
for i=1:2
    l = sqrt(sum(diff(seg{i}).^2,2)); % Segment lengths
    c = (seg{i}(2:end,:)+seg{i}(1:end-1,:))/2; % Segment centroids
    
    capInertia = capInertia + abs(sum(l.*c(:,2).^2)); % Flap Inertia per unit thickness
    capLength = capLength + sum(l);
end

% Shear Properties
box = coord; % Get airfoil coordinates
box(box(:,1)>fwdWeb(2),:) = []; % Trim coordinates
z = box(box(:,1)==fwdWeb(1),2)*chord;
shearHeight = abs(z(1)-z(2));

% Core Properties
box = coord; % Get airfoil coordinates
box(box(:,1)<fwdWeb(1),:) = [];
box = box*chord;
coreArea = polyarea(box(:,1),box(:,2));

%% Shear/Moment Calcs
Vz=[cumsum(F(1:end-1).*diff(x),'reverse') 0]; % Shear due to lift
Mx=[cumsum(Vz(1:end-1).*diff(x),'reverse') 0]; % Flap moment
My=[cumsum( Q(1:end-1).*diff(x),'reverse') 0]; % Torsion moment

while error > tolerance
    CF=sf*omega^2*[cumsum(m(1:end-1).*diff(x).*x(1:end-1),'reverse') 0]; % Centripetal force
    
    %% Structural Calcs
    % Torsion Analysis: all torsion taken in skin
    tTorsion = My/(2*bid.shear*Ae); % Torsion skin thickness
    tTorsion = max(tTorsion,bid.minThk*ones(1,N)); % Min gauge constraint
    mTorsion = tTorsion*skinLength*bid.rho; % Mass for torsion
    
    % Flap Bending Analysis: all bending taken in fwd caps
    tFlap = CF/(capLength*uni.stress) + Mx*max(abs(box(:,2)))/(capInertia*uni.stress); % Thickness for flap bending
    mFlap = tFlap*capLength*uni.rho; % Mass for flap bending
    mGlue = glue.thk*glue.rho*capLength*ones(1,N);
    
    % Shear Web Analysis: all shear taken in shear web
    tShear = 1.5*Vz/(bid.shear*shearHeight);
    tShear = max(tShear,bid.minThk*ones(1,N)); % Min gauge constraint
    mShear = tShear*shearHeight*bid.rho;
    
    % Paint weight
    mPaint = skinLength*paint.thk*paint.rho*ones(1,N);
    
    % Core Mass
    mCore = coreArea*core.rho*ones(1,N);
    mGlue = mGlue+glue.thk*glue.rho*skinLength*ones(1,N);
    
    % Section mass
    m = mTorsion+mCore+mFlap+mShear+mGlue+mPaint;
 
    % Rib weight
    mRib = (Ae+skinLength*rib.width)*rib.thk*alum.rho;
    
    % Root fitting
    box = coord*chord;
    rRoot = max(box(:,2))-min(box(:,2))/2; % Fitting diam is thickness
    t = max(CF)/(2*pi*rRoot*alum.stress) + max(Mx)/(3*pi*rRoot^2*alum.stress);
    mRoot = 2*pi*rRoot*t*rootLength*alum.rho;
    
    % Total weight
    mass = nBlades*(sum(m(1:end-1).*diff(x))+2*mRib+mRoot);
    error = abs(mass-massOld);
    massOld = mass;
end

mass = fudge*mass; % Fudge factor
