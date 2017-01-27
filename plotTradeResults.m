% Function that loads the configuration sizing trade study results and
% generates some interesting plots 

function [] = plotTradeResults()

% Constants
km2m = 1000;

%% Load trade study data
load('tradeStudyResults.mat');

%% Save data in usable format
ranges = ranges';
mH = nan(length(ranges),1); % Helicopter mass
mT = nan(length(ranges),1); % Tilt-wing mass
mBatH = nan(length(ranges),1); % Helicopter battery mass
mBatT = nan(length(ranges),1); % Tilt-wing battery mass
vH = nan(length(ranges),1); % Helicopter cruise speed
vT = nan(length(ranges),1); % Tilt-wing cruise speed
bH = nan(length(ranges),1); % Helicopter span
bT = nan(length(ranges),1); % Tilt-wing span
lH = nan(length(ranges),1); % Helicopter length
lT = nan(length(ranges),1); % Tilt-wing length
cH = nan(length(ranges),1); % Helicopter DOC per flight
cT = nan(length(ranges),1); % Tilt-wing DOC per flight
rH = nan(length(ranges),1); % Helicopter rotor radius
rT = nan(length(ranges),1); % Tilt-wing fan radius
pHH = nan(length(ranges),1); % Helicopter hover power
pHT = nan(length(ranges),1); % Tilt-wing hover power
pCH = nan(length(ranges),1); % Helicopter cruise power
pCT = nan(length(ranges),1); % Tilt-wing cruise power
mBreakdownH = nan(length(ranges),6); % Helicopter mass breakdown
mBreakdownT = nan(length(ranges),6); % Tilt-wing mass breakdown
CBreakdownH = nan(length(ranges),6); % Helicopter cost breakdown
CBreakdownT = nan(length(ranges),6); % Tilt-wing cost breakdown

% Loop through results and pack vectors
for i = 1:length(ranges)
    if length(CHelicopter) >= i
        if ~isempty(massHelicopter(i).m) 
            mH(i) = massHelicopter(i).m; 
        end
        if ~isempty(massHelicopter(i).battery)
            mBatH(i) = massHelicopter(i).battery; 
        end
        vH(i) = VHelicopter(i);
        bH(i) = 2 * rPropHelicopter(i);
        lH(i) = 2.25 * rPropHelicopter(i);
        rH(i) = rPropHelicopter(i);
        pHH(i) = hoverOutputHelicopter(i).PBattery * 1e-3;
        pCH(i) = cruiseOutputHelicopter(i).PBattery * 1e-3;
        mBreakdownH(i,:) = 1.1*[massHelicopter(i).payload, massHelicopter(i).avionics + ...
            massHelicopter(i).servos + massHelicopter(i).wire, ...
            massHelicopter(i).seat + massHelicopter(i).brs, ...
            massHelicopter(i).battery, massHelicopter(i).motors + massHelicopter(i).transmission, ...
             massHelicopter(i).structural];
        CBreakdownH(i,:) = [CHelicopter(i).acquisitionCostPerFlight,...
            CHelicopter(i).insuranceCostPerFlight,...
            CHelicopter(i).facilityCostPerFlight,...
            CHelicopter(i).energyCostPerFlight,...
            CHelicopter(i).batteryReplCostPerFlight + CHelicopter(i).motorReplCostPerFlight + CHelicopter(i).servoReplCostPerFlight, ...
            CHelicopter(i).laborCostPerFlight];
        if ~isempty(CHelicopter(i).costPerFlight); cH(i) = CHelicopter(i).costPerFlight; end;
    end
    if length(CTiltWing) >= i
        if ~isempty(massTiltWing(i).m)
            mT(i) = massTiltWing(i).m; 
        end
        if ~isempty(massTiltWing(i).battery)
            mBatT(i) = massTiltWing(i).battery; 
        end
        vT(i) = VTiltWing(i);
        bT(i) = 8 * rPropTiltWing(i) + 1;
        lT(i) = 4 * rPropTiltWing(i) + 3;
        rT(i) = rPropTiltWing(i);
        pHT(i) = hoverOutputTiltWing(i).PBattery * 1e-3;
        pCT(i) = cruiseOutputTiltWing(i).PBattery * 1e-3;
        mBreakdownT(i,:) = 1.1*[massTiltWing(i).payload, massTiltWing(i).avionics + ...
            massTiltWing(i).servos + massTiltWing(i).wire + massTiltWing(i).tilt, ...
            massTiltWing(i).seat + massTiltWing(i).brs, ...
            massTiltWing(i).battery, massTiltWing(i).motors, ...
             massTiltWing(i).structural];
         CBreakdownT(i,:) = [CTiltWing(i).acquisitionCostPerFlight,...
            CTiltWing(i).insuranceCostPerFlight,...
            CTiltWing(i).facilityCostPerFlight,...
            CTiltWing(i).energyCostPerFlight,...
            CTiltWing(i).batteryReplCostPerFlight + CTiltWing(i).motorReplCostPerFlight + CTiltWing(i).servoReplCostPerFlight, ...
            CTiltWing(i).laborCostPerFlight];
        if ~isempty(CTiltWing(i).costPerFlight); cT(i) = CTiltWing(i).costPerFlight; end;
    end
end



%% DOC per flight (dot area represents mass)
figuren('Cost vs. Range'); clf; hold on;
plot(ranges/km2m,cH,'b-')
plot(ranges/km2m,cT,'r-')
for i = 1:length(ranges)
    if ~isnan(cH(i))
        plot(ranges(i)/km2m,cH(i),'b.','MarkerSize',1.5*sqrt(mH(i)))
    end
    if ~isnan(cT(i))
        plot(ranges(i)/km2m,cT(i),'r.','MarkerSize',1.5*sqrt(mT(i)))
    end
end
grid on;
xlabel('Range [km]')
ylabel('DOC [$]')
ylim([0,450])
xlim([0,1.1*max(ranges)/1000]) 
legend('Electric Helicopter','Electric Tilt-Wing Multirotor','Location','Best')
plot(10,165,'k.','MarkerSize',1.5*sqrt(300));
plot(10,140,'k.','MarkerSize',1.5*sqrt(600));
plot(10,115,'k.','MarkerSize',1.5*sqrt(900));
plot(10,90,'k.','MarkerSize',1.5*sqrt(1200));
plot(10,65,'k.','MarkerSize',1.5*sqrt(1500));
text(15,165,'300 kg')
text(15,140,'600 kg')
text(15,115,'900 kg')
text(15,90,'1200 kg')
text(15,65,'1500 kg')


%% Cost per km vs. range
figuren('Cost per km vs. Range'); clf; hold on;
plot(ranges/km2m,cH./(ranges/km2m),'b-')
plot(ranges/km2m,cT./(ranges/km2m),'r-')
for i = 1:length(ranges)
    if ~isnan(cH(i))
        plot(ranges(i)/km2m,cH(i)/(ranges(i)/km2m),'b.','MarkerSize',1.5*sqrt(mH(i)))
    end
    if ~isnan(cT(i))
        plot(ranges(i)/km2m,cT(i)/(ranges(i)/km2m),'r.','MarkerSize',1.5*sqrt(mT(i)))
    end
end
grid on;
xlabel('Range [km]')
ylabel('DOC [$/km]')
ylim([0,3.5])
xlim([0,1.1*max(ranges)/1000]) 
legend('Electric Helicopter','Electric Tilt-Wing Multirotor','Location','Best')
plot(50,3.0,'k.','MarkerSize',1.5*sqrt(300));
plot(50,2.7,'k.','MarkerSize',1.5*sqrt(600));
plot(50,2.4,'k.','MarkerSize',1.5*sqrt(900));
plot(50,2.1,'k.','MarkerSize',1.5*sqrt(1200));
plot(50,1.8,'k.','MarkerSize',1.5*sqrt(1500));
text(55,3.0,'300 kg')
text(55,2.7,'600 kg')
text(55,2.4,'900 kg')
text(55,2.1,'1200 kg')
text(55,1.8,'1500 kg')
saveas(gcf,'./DOCvsRange','png')


%% DOC (dot area represents size)
figuren('Cost vs. Range - Size'); clf; 
subplot(2,1,1); hold on; 
plot(ranges/km2m,cH,'b-')
plot(ranges/km2m,cT,'r-')
for i = 1:length(ranges)
    if ~isnan(cT(i))
        plot(ranges(i)/km2m,cT(i),'r.','MarkerSize',7 * sqrt(bT(i) * lT(i)))
    end
    if ~isnan(cH(i))
        plot(ranges(i)/km2m,cH(i),'b.','MarkerSize',7 * sqrt(bH(i) * lH(i)))
    end
end
grid on;
xlabel('Range [km]')
ylabel('DOC [$]')
ylim([0,450])
xlim([0,1.1*max(ranges)/1000]) 
legend('Electric Helicopter','Electric Tilt-Wing Multirotor','Location','Best')
plot(10,165,'k.','MarkerSize',7 * sqrt(20));
plot(10,140,'k.','MarkerSize',7 * sqrt(40));
plot(10,115,'k.','MarkerSize',7 * sqrt(60));
plot(10,90,'k.','MarkerSize',7 * sqrt(80));
text(15,165,'20 m^2')
text(15,140,'40 m^2')
text(15,115,'60 m^2')
text(15,90,'80 m^2')

% Cost per km vs. range
subplot(2,1,2); hold on;
plot(ranges/km2m,cH./(ranges/km2m),'b-')
plot(ranges/km2m,cT./(ranges/km2m),'r-')
for i = 1:length(ranges)
    if ~isnan(cT(i))
        plot(ranges(i)/km2m,cT(i)/(ranges(i)/km2m),'r.','MarkerSize',7 * sqrt(bT(i) * lT(i)))
    end
    if ~isnan(cH(i))
        plot(ranges(i)/km2m,cH(i)/(ranges(i)/km2m),'b.','MarkerSize',7 * sqrt(bH(i) * lH(i)))
    end
end
grid on;
xlabel('Range [km]')
ylabel('DOC [$/km]')
ylim([0,3.5])
xlim([0,1.1*max(ranges)/1000]) 
legend('Electric Helicopter','Electric Tilt-Wing Multirotor','Location','Best')


%% Relative Cost
figuren('Cost Deltas vs. Range'); clf;
% Plot ratio of costs vs. range
subplot(2,1,1); hold on;
plot(ranges/km2m,cT./cH,'b.-','MarkerSize',20)
grid on
xlabel('Range [km]')
ylabel('DOC Tilt-Wing / DOC Helicopter');
ylim([0,1.3])

% Plot delta cost vs. range
subplot(2,1,2); hold on;
plot(ranges/km2m,cT-cH,'b.-','MarkerSize',20)
grid on
xlabel('Range [km]')
ylabel('DOC Tilt-Wing - DOC Helicopter');
ylim([-40,20])


%% Vehicle Size
figuren('Vehicle Size'); clf;

% Prop/Rotor Radius
subplot(2,2,1); hold on;
plot(ranges/km2m, rH, 'b.-')
plot(ranges/km2m, rT, 'r.-')
ylim([0,1.1*max([rH;rT])])
grid on
xlabel('Range [km]')
ylabel('Prop/Rotor Radius [m]')
legend('Electric Helicopter','Electric Tilt-Wing Multirotor','Location','Best')

% Span
subplot(2,2,2); hold on;
plot(ranges/km2m, bH, 'b.-')
plot(ranges/km2m, bT, 'r.-')
ylim([0,1.1*max([bH;bT])])
grid on
xlabel('Range [km]')
ylabel('Span [m]')

% Area
subplot(2,2,3); hold on;
plot(ranges/km2m, bH.*lH, 'b.-')
plot(ranges/km2m, bT.*lT, 'r.-')
ylim([0,1.1*max([bH.*lH;bT.*lT])])
grid on
xlabel('Range [km]')
ylabel('Footprint [m^2]')

% Length
subplot(2,2,4); hold on;
plot(ranges/km2m, lH, 'b.-')
plot(ranges/km2m, lT, 'r.-')
ylim([0,1.1*max([lH;lT])])
grid on
xlabel('Range [km]')
ylabel('Length [m]')


%% Plot hover power, cruise power, disk loading
figuren('Power and Loading'); clf;

% Hover Power (battery power)
subplot(2,2,1); hold on; 
plot(ranges/km2m, pHH, 'b.-')
plot(ranges/km2m, pHT, 'r.-')
ylim([0,1.1*max([pHH;pHT])])
grid on
xlabel('Range [km]')
ylabel('Hover Electrical Power [kW]')
legend('Electric Helicopter','Electric Tilt-Wing Multirotor','Location','Best')

% Disk Loading (Thrust/Area)
subplot(2,2,2); hold on; 
plot(ranges/km2m, mH * 9.8 ./ (pi * rH.^2), 'b.-')
plot(ranges/km2m, mT * 9.8 ./ (8 * pi * rT.^2), 'r.-')
ylim([0,1.1*max([mH * 9.8 ./ (pi * rH.^2);mT * 9.8 ./ (8 * pi * rT.^2)])])
grid on
xlabel('Range [km]')
ylabel('Hover Disk Loading [N/m^2]')

% Cruise Power (power)
subplot(2,2,3); hold on; 
plot(ranges/km2m, pCH, 'b.-')
plot(ranges/km2m, pCT, 'r.-')
ylim([0,1.1*max([pCH;pCT])])
grid on
xlabel('Range [km]')
ylabel('Cruise Electrical Power [kW]')

% Power Loading (mass / power)
subplot(2,2,4); hold on; 
plot(ranges/km2m, mH ./ pHH, 'b.-')
plot(ranges/km2m, mT ./ pHT, 'r.-')
ylim([0,1.1*max([mH ./ pHH;mT ./ pHT])])
grid on
xlabel('Range [km]')
ylabel('Hover Power Loading [kg/kW]')
saveas(gcf,'./powerAndLoading','png');


%% Mass Breakdown
figuren('Mass Breakdown'); clf;
subplot(2,1,1); hold on;
bar(ranges/km2m, mBreakdownH,'stacked')
xlim([0,210])
ylim([0,2000])
grid on
xlabel('Range [km]')
ylabel('Mass [kg]')
title('Electric Helicopter')
legend('Payload','Avionics','Misc','Battery','Motors+Transmission','Structure','Location','Best')

subplot(2,1,2); hold on;
bar(ranges/km2m, mBreakdownT, 'stacked')
xlim([0,210])
ylim([0,2000])
grid on
xlabel('Range [km]')
ylabel('Mass [kg]')
title('Electric Tilt-Wing Multirotor')
saveas(gcf,'./massBreakdown','png');


%% Cost Breakdown
figuren('Cost Breakdown'); clf;
subplot(2,1,1); hold on;
bar(ranges/km2m, CBreakdownH ./ (repmat(ranges,1,6)/km2m),'stacked')
xlim([0,210])
ylim([0,3.5])
grid on
xlabel('Range [km]')
ylabel('DOC [$/km]')
title('Electric Helicopter')
legend('Aquisition','Insurance','Facility','Energy','Part Replacement','Labor','Location','Best')

subplot(2,1,2); hold on;
bar(ranges/km2m, CBreakdownT ./ (repmat(ranges,1,6)/km2m), 'stacked')
xlim([0,210])
ylim([0,3.5])
grid on
xlabel('Range [km]')
ylabel('DOC [$/km]')
title('Electric Tilt-Wing Multirotor')
saveas(gcf,'./costBreakdown','png');


