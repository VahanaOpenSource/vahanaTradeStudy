% Plot results of sensitivity studies for 50 km range tilt-wing design

%% Battery specific energy density
E =   [150,    230,   350,   500  ];
m =   [1008.6, 703.2, 559.8, 495.0];
DOC = [88.53,  61.97, 49.07, 42.57] / 50;

figuren('Specific Energy Density'); clf; 
hold on;
yyaxis left
plot(E,DOC,'.-')
plot(E(2),DOC(2),'o')
ylabel('DOC [$/km]')
ylim([0,2])
yyaxis right
plot(E,m,'.-')
plot(E(2),m(2),'o')
ylabel('Takeoff Mass [kg]')
ylim([0,1100])
grid on
xlabel('Specific Energy Density [W-hr/kg]')
xlim([0,550])
saveas(gcf,'./energyDensitySensitivity','png')


%% Battery cycle life
cycles = [500,    1000,  2000,  5000 ];
m =      [692.0,  694.2, 703.2, 732.5];
DOC =    [117.87, 81.01, 61.97, 49.97] / 50;

figuren('Battery Cycle Life'); clf; 
hold on;
plot(cycles,DOC,'.-')
plot(cycles(3),DOC(3),'bo')
ylabel('DOC [$/km]')
grid on
xlabel('Battery Life [Cycles]')
xlim([0,5500])
ylim([0,2.5])
saveas(gcf,'./cycleLifeSensitivity','png')


%% Reserve segment duration
% Loiter time is tReserve - 3 min for hover/transition
tReserve = [5,     10,    20,    30,    45   ];
m =        [595.8, 630.1, 703.2, 782.7, 916.2];
DOC =      [52.18, 55.33, 61.97, 69.12, 80.99] / 50;

figuren('Reserve Segment Duration'); clf; 
hold on;
yyaxis left
plot(tReserve,DOC,'.-')
plot(tReserve(3),DOC(3),'o')
ylabel('DOC [$/km]')
ylim([0,2])
yyaxis right
plot(tReserve,m,'.-')
plot(tReserve(3),m(3),'o')
ylabel('Takeoff Mass [kg]')
ylim([0,1000])
grid on
xlabel('Reserve Segment Duration [min]')
xlim([0,50])
saveas(gcf,'./reserveMissionSensitivity','png')

