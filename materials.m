%% List of Material properties
% rho is density [kg/m^3]
% stress is design ultimate tensile stress [Pa]
% shear is design ultimate shear stress [Pa]
% minThk is minimum gauge thickness [m]
% width is rib width
% bearing is bearing allowable [Pa]

% Unidirectional carbon fiber
uni.rho = 1660;
uni.stress = 450e6;

% Bi-directional carbon fiber
bid.rho = 1660;
bid.stress = 275e6;
bid.shear = 47e6;
bid.minThk = 0.00042;
bid.bearing = 400e6;

% Honeycomb core
core.rho = 52;
core.minThk = 0.0064;

% Epoxy
glue.thk = 2.54e-4;
glue.rho = 1800;

% Aluminum ribs
rib.thk = 0.0015;
rib.width = 0.0254;

% Paint or vinyl
paint.thk = 0.00015;
paint.rho = 1800;

% Aluminum
alum.stress = 350e6;
alum.rho = 2800;

% Acrylic
canopy.thk = 0.003175;
canopy.rho = 1180;

% Steel
steel.shear = 500e6;