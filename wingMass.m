% Estimate lifting surface structural mass 
%
% Inputs:
% W       - vehicle weight (to calculate lift) [N]
% span    - wing span [m]
% chord   - wing chord [m]
% winglet - winglet length normalized by semi-span
% fc      - fraction of vehicle lift supported by wing
% xmotor  - vector of motor locations normalized by semi-span
% thrust  - maximum thrust produced by motor
%
% Outputs:
% mass - Mass of the lifting surface
%


function mass = wingMass(W,span,chord,winglet,fc,xmotor,thrust)

%% Setup
N = 10; % Number of spanwise points
sf = 1.5; % Safety factor
n = 3.8; % Maximum g's
toc = 0.15; % Airfoil thickness
cmocl = 0.02/1; % Ratio of cm/cl for sizing torsion (magnitude)
LoD = 7; % For drag loads
fwdWeb = [0.25 0.35]; % Forward web location x/c
aftWeb = [0.65 0.75]; % Aft web location x/c
xShear = 0.25; % Approximate shear center
fudge = 1.2; % Scale up mass by this to account for misc components

nRibs = length(xmotor)+2;
xmotor = xmotor*span/2;

%% Material properties
materials;

%% Airfoil
naca = 5*toc*[0.2969 -0.1260 -0.3516 0.2843 -0.1015]'; % Thickness distribution for NACA 4-series airfoil
coord = unique([fwdWeb aftWeb linspace(0,1,N)])';
coord(:,2) = [coord(:,1).^0.5 coord(:,1) coord(:,1).^2 coord(:,1).^3 coord(:,1).^4]*naca;
coord = [flipud(coord(2:end,:)); coord*[1 0;0 -1]];
coord(:,1) = coord(:,1)-xShear;

%% Beam Geometry
x = [linspace(0,1,N) linspace(1,1+winglet,N)]*span/2;
x = sort([x xmotor]);
dx = x(2)-x(1);
N = length(x);
fwdWeb = fwdWeb - xShear;
aftWeb = aftWeb - xShear;

%% Loads
L = (1-(x/max(x)).^2).^(1/2); % Elliptic lift distribution profile
L0 = 0.5*n*W*fc*sf; % Total design lift force on surface
L = L0/sum(L(1:N-1).*diff(x(1:N)))*L; % Lift distribution

T = L*chord*cmocl; % Torque distribution
D = L/LoD; % Drag distribution

%% Shear/Moment Calcs
Vx=[cumsum(D(1:end-1).*diff(x),'reverse') 0]; % Shear due to drag
Vz=[cumsum(L(1:end-1).*diff(x),'reverse') 0]; % Shear due to lift
Vt=0*Vz; % Initialize shear due to thrust
for i=1:length(xmotor)
    Vt(x<=xmotor(i)) = Vt(x<=xmotor(i))+thrust; % Shear due to thrust
end

Mx=[cumsum(Vz(1:end-1).*diff(x),'reverse') 0]; % Bending moment
My=[cumsum( T(1:end-1).*diff(x),'reverse') 0]; % Torsion moment
Mz=[cumsum(Vx(1:end-1).*diff(x),'reverse') 0]; % Drag moment
Mt=[cumsum(Vt(1:end-1).*diff(x),'reverse') 0]; % Thrust moment
Mz=max(Mz,Mt); %Worst case Mz

%% General structural properties
% Torsion Cell
box = coord;
box(box(:,1)>aftWeb(2),:) = [];
box(box(:,1)<fwdWeb(1),:) = [];
box = box*chord;
torsionArea = polyarea(box(:,1),box(:,2)); % Enclosed wing area
torsionLength = sum(sqrt(sum(diff(box).^2,2)));

% Bending
box = coord; % Get airfoil coordinates
box(box(:,1)>fwdWeb(2),:) = [];
box(box(:,1)<fwdWeb(1),:) = []; % Trim coordinates
seg{1} = box(box(:,2)>mean(box(:,2)),:)*chord; % Upper fwd segment
seg{2} = box(box(:,2)<mean(box(:,2)),:)*chord; % Lower fwd segment

% Drag
box = coord; % Get airfoil coordinates
box(box(:,1)>aftWeb(2),:) = [];
box(box(:,1)<aftWeb(1),:) = []; % Trim coordinates
seg{3} = box(box(:,2)>mean(box(:,2)),:)*chord; % Upper aft segment
seg{4} = box(box(:,2)<mean(box(:,2)),:)*chord; % Lower aft segment

% Bending/drag inertia
flapInertia=0; flapLength=0; dragInertia=0; dragLength=0;
for i=1:4
    l = sqrt(sum(diff(seg{i}).^2,2)); % Segment lengths
    c = (seg{i}(2:end,:)+seg{i}(1:end-1,:))/2; % Segment centroids
    
    if i<3
        flapInertia = flapInertia + abs(sum(l.*c(:,2).^2)); % Bending Inertia per unit thickness
        flapLength = flapLength + sum(l);
    else
        dragInertia = dragInertia + abs(sum(l.*c(:,1).^2)); % Drag Inertia per unit thickness
        dragLength = dragLength + sum(l);
    end
end

% Shear
box = coord; % Get airfoil coordinates
box(box(:,1)>fwdWeb(2),:)=[]; % Trim coordinates
z(1) = interp1(box(box(:,2)>0,1),box(box(:,2)>0,2),fwdWeb(1))*chord;
z(2) = interp1(box(box(:,2)<0,1),box(box(:,2)<0,2),fwdWeb(1))*chord;
h = z(1)-z(2);

% Skin
box = coord*chord;
skinLength = sum(sqrt(sum(diff(box).^2,2)));
A = polyarea(box(:,1),box(:,2));

%% Structural Calcs

% Torsion Analysis: all torsion taken in skin
tTorsion = My*dx/(2*bid.shear*torsionArea); % Torsion skin thickness
tTorsion = max(tTorsion,bid.minThk*ones(1,N)); % Min gauge constraint
mTorsion = tTorsion*torsionLength*bid.rho; % Mass for torsion
mCore = core.minThk*torsionLength*core.rho*ones(1,N); % Core mass
mGlue = glue.thk*glue.rho*torsionLength*ones(1,N);

% Flap Bending Analysis
tFlap = Mx*max(seg{1}(:,2))/(flapInertia*uni.stress); % Thickness for flap bending
mFlap = tFlap*flapLength*uni.rho; % Mass for flap bending
mGlue = mGlue+glue.thk*glue.rho*flapLength*ones(1,N);

% Drag Bending Analysis
tDrag = Mz*max(seg{3}(:,1))/(dragInertia*uni.stress); % Thickness for flap bending
mDrag = tDrag*dragLength*uni.rho; % Mass for flap bending
mGlue = mGlue + glue.thk*glue.rho*dragLength*ones(1,N);

% Shear Web Analysis: all shear taken in shear web
tShear = 1.5*Vz/(bid.shear*h);
tShear = max(tShear,bid.minThk*ones(1,N)); % Min gauge constraint
mShear = tShear*h*bid.rho;

% Paint weight
mPaint = skinLength*paint.thk*paint.rho*ones(1,N);

% Section mass
m = mTorsion+mCore+mFlap+mDrag+mShear+mGlue+mPaint;

% Rib weight
mRib = (A+skinLength*rib.width)*rib.thk*alum.rho;

% Total weight
mass = 2*(sum(m(1:end-1).*diff(x))+nRibs*mRib)*fudge;