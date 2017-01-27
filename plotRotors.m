% Plot different diameter circles with the same disk area to illustrate
% packing efficiency

r = 1;

theta = 0:360;
x = cosd(theta);
y = sind(theta);

figuren('Packing Efficiency'); clf; hold on;

% 1 rotor
% subplot(1,5,1); hold on;
patch(r * x, r * y,'g','FaceAlpha',1.00,'EdgeAlpha',0.3);
xlim([-2.25,2.25])
axis equal
set(gca,'visible','off')

% 4 rotors
% subplot(1,5,2); hold on;
r4 = sqrt(r^2 / 4);
xc = [-r4,r4,-r4,r4]+[-1 1 -1 1]*.25;
yc = [-r+r4, -r+r4, r-r4, r-r4];
for i = 1:length(xc)
    patch(xc(i) + r4 * x, yc(i) + r4 * y, 'c','FaceAlpha',1.00,'EdgeAlpha',0.3)
end
xlim([-2.25,2.25])
axis equal
set(gca,'visible','off')

% 8 rotors
% subplot(1,5,3); hold on;
r8 = sqrt(r^2 / 8);
xc = [-3*r8,-r8,r8,3*r8,-3*r8,-r8,r8,3*r8]+[-1 -1 1 1 -1 -1 1 1]*.25;
yc = [-r+r8,-r+r8,-r+r8,-r+r8,r-r8,r-r8,r-r8,r-r8];
for i = 1:length(xc)
    patch(xc(i) + r8 * x, yc(i) + r8 * y, 'b','FaceAlpha',1.00,'EdgeAlpha',0.3)
end
xlim([-2.25,2.25])
axis equal
set(gca,'visible','off')

% 12 rotors
% subplot(1,5,4); hold on;
r12 = sqrt(r^2 / 12);
xc = [-5*r12,-3*r12,-r12,r12,3*r12,5*r12,-5*r12,-3*r12,-r12,r12,3*r12,5*r12]+[-1 -1 -1 1 1 1 -1 -1 -1 1 1 1]*.25;
yc = [-r+r12*ones(1,6),r-r12*ones(1,6)];
for i = 1:length(xc)
    patch(xc(i) + r12 * x, yc(i) + r12 * y, 'm','FaceAlpha',1.00,'EdgeAlpha',0.3)
end
xlim([-2.25,2.25])
axis equal
set(gca,'visible','off')

% 16 rotors
% subplot(1,5,5); hold on;
r16 = sqrt(r^2 / 16);
xc = [-7*r16,-5*r16,-3*r16,-r16,r16,3*r16,5*r16,7*r16,-7*r16,-5*r16,-3*r16,-r16,r16,3*r16,5*r16,7*r16]+[-1 -1 -1 -1 1 1 1 1 -1 -1 -1 -1 1 1 1 1]*.25;
yc = [-r+r16*ones(1,8),r-r16*ones(1,8)];
for i = 1:length(xc)
    patch(xc(i) + r16 * x, yc(i) + r16 * y, 'r','FaceAlpha',1.00,'EdgeAlpha',0.3)
end
xlim([-2.25,2.25])
axis equal
set(gca,'visible','off')
alpha(.6)

