% Robust control of wall-driven swimmer

clear all;

% Initialisation

k = 1;%flow length scale
w = 6;%flow frequency
v = 1;%swimming speed
umax = 5;%flow amplitude
a = 0.4;%swimmer radius

L = 2*pi/k;
tau = 2*pi/w;

% initial condition
x0 = 0;
z0 = 3;
th0 = -pi/2;

% final time
T = 5;
tps = linspace(0,T,100);

%% 1. open loop control

B2=0;% no squirmer term

% switching time
V0=umax*exp(-k*z0)*(1-k*z0);
T_switch = pi/(w-v*sin(th0)-2*V0/pi);

% prescribed control
u = @(t,x) umax*sign(sin(pi*t/T_switch));

% dynamics

swim = @(t,Y) [-v*sin(Y(3));...
    v*cos(Y(3));...
    0];

wall = @(t,Y) [u(t,Y(1))*exp(-k*Y(2))*(1-k*Y(2))*sin(k*Y(1)-w*t);
    -u(t,Y(1))*exp(-k*Y(2))*k*Y(2)*cos(k*Y(1)-w*t);
    u(t,Y(1))*exp(-k*Y(2))*k*sin(k*Y(1)-w*t)/2];

squirm = @(t,Y) [3*a*a*sin(2*Y(3))*(-B2)/5/8/Y(2)^2;
    -3*a*a*(1+3*cos(2*Y(3))*(-B2)/5)/16/Y(2)^2;
    3*a*a*sin(2*Y(3))*(-B2)/5/16/Y(2)^3];

f = @(t,Y) swim(t,Y) + wall(t,Y) + squirm(t,Y);

[tps1,traj1]=ode23(f,tps,[x0 z0 th0]);

%% 2. feedback control

B2=0;% no squirmer term

% prescribed control
u = @(t,x) -umax*sign(sin(k*x-w*t));

% dynamics

swim = @(t,Y) [-v*sin(Y(3));...
    v*cos(Y(3));...
    0];

wall = @(t,Y) [u(t,Y(1))*exp(-k*Y(2))*(1-k*Y(2))*sin(k*Y(1)-w*t);
    -u(t,Y(1))*exp(-k*Y(2))*k*Y(2)*cos(k*Y(1)-w*t);
    u(t,Y(1))*exp(-k*Y(2))*k*sin(k*Y(1)-w*t)/2];

squirm = @(t,Y) [3*a*a*sin(2*Y(3))*(-B2)/5/8/Y(2)^2;
    -3*a*a*(1+3*cos(2*Y(3))*(-B2)/5)/16/Y(2)^2;
    3*a*a*sin(2*Y(3))*(-B2)/5/16/Y(2)^3];

f = @(t,Y) swim(t,Y) + wall(t,Y) + squirm(t,Y);

[tps2,traj2]=ode23(f,tps,[x0 z0 th0]);

%% 3. feedback + squirmer term pusher

B2=30;

% prescribed control
u = @(t,x) -umax*sign(sin(k*x-w*t));

% dynamics

swim = @(t,Y) [-v*sin(Y(3));...
    v*cos(Y(3));...
    0];

wall = @(t,Y) [u(t,Y(1))*exp(-k*Y(2))*(1-k*Y(2))*sin(k*Y(1)-w*t);
    -u(t,Y(1))*exp(-k*Y(2))*k*Y(2)*cos(k*Y(1)-w*t);
    u(t,Y(1))*exp(-k*Y(2))*k*sin(k*Y(1)-w*t)/2];

squirm = @(t,Y) [3*a*a*sin(2*Y(3))*(-B2)/5/8/Y(2)^2;
    -3*a*a*(1+3*cos(2*Y(3))*(-B2)/5)/16/Y(2)^2;
    3*a*a*sin(2*Y(3))*(-B2)/5/16/Y(2)^3];

f = @(t,Y) swim(t,Y) + wall(t,Y) + squirm(t,Y);

[tps3,traj3]=ode23(f,tps,[x0 z0 th0]);

%% 3. feedback + squirmer term puller

B2=-20;

% prescribed control
u = @(t,x) -umax*sign(sin(k*x-w*t));

% dynamics

swim = @(t,Y) [-v*sin(Y(3));...
    v*cos(Y(3));...
    0];

wall = @(t,Y) [u(t,Y(1))*exp(-k*Y(2))*(1-k*Y(2))*sin(k*Y(1)-w*t);
    -u(t,Y(1))*exp(-k*Y(2))*k*Y(2)*cos(k*Y(1)-w*t);
    u(t,Y(1))*exp(-k*Y(2))*k*sin(k*Y(1)-w*t)/2];

squirm = @(t,Y) [3*a*a*sin(2*Y(3))*(-B2)/5/8/Y(2)^2;
    -3*a*a*(1+3*cos(2*Y(3))*(-B2)/5)/16/Y(2)^2;
    3*a*a*sin(2*Y(3))*(-B2)/5/16/Y(2)^3];

f = @(t,Y) swim(t,Y) + wall(t,Y) + squirm(t,Y);

[tps4,traj4]=ode23(f,tps,[x0 z0 th0]);

%% 3. feedback + squirmer term strong puller

B2=-100;

% prescribed control
u = @(t,x) -umax*sign(sin(k*x-w*t));

% dynamics

swim = @(t,Y) [-v*sin(Y(3));...
    v*cos(Y(3));...
    0];

wall = @(t,Y) [u(t,Y(1))*exp(-k*Y(2))*(1-k*Y(2))*sin(k*Y(1)-w*t);
    -u(t,Y(1))*exp(-k*Y(2))*k*Y(2)*cos(k*Y(1)-w*t);
    u(t,Y(1))*exp(-k*Y(2))*k*sin(k*Y(1)-w*t)/2];

squirm = @(t,Y) [3*a*a*sin(2*Y(3))*(-B2)/5/8/Y(2)^2;
    -3*a*a*(1+3*cos(2*Y(3))*(-B2)/5)/16/Y(2)^2;
    3*a*a*sin(2*Y(3))*(-B2)/5/16/Y(2)^3];

f = @(t,Y) swim(t,Y) + wall(t,Y) + squirm(t,Y);

[tps5,traj5]=ode23(f,tps,[x0 z0 th0]);

%% post-processing

z_max = 6/k;
dz = z_max/20;
x_min = (min(x0,traj1(end,1)) - L/4);
x_max = (max(x0,traj1(end,1)) + L/2);
dx = (x_max-x_min)/50;
[X,Z] = meshgrid(x_min:dx:x_max,0:dz:z_max);

wall_x = @(x,z,u,t) u*exp(-k*z).*(1-k*z).*sin(k*x-w*t);
wall_z = @(x,z,u,t) -u*k*exp(-k*z).*z.*cos(k*x-w*t);
wall_th = @(x,z,u,t) u*k*exp(-k*z).*sin(k*x-w*t)/2;

figure(1);clf
pl=tiledlayout(1,5);

nexttile;

%flow 
u = @(t,x) umax*sign(sin(pi*t/T_switch));
Ux = wall_x(X,Z,u(T,0),T);
Uz = wall_z(X,Z,u(T,0),T);
c = ceil(99*(u(T,0)/umax+1)/2)+1;
even_stream_arrow(X, Z, Ux, Uz, 1, 2, ...
    'LineStyle', '-', 'LineWidth', 0.5, 'Color', [0 0.77 0.93], 'ArrowLength', 4, ...
    'ArrowTipAngle', 20, 'ArrowBaseAngle', 60, 'ArrowDensity', 1);
alpha(.4)
hold on

% trajectory
plot(traj1(1:end,1),traj1(1:end,2),'k','LineWidth',2)
hold on

% swimmer, orientation
swimmer = nsidedpoly(30,'Center',[traj1(end,1) traj1(end,2)],'Radius',a);
plot(swimmer,'FaceAlpha',1,'FaceColor',[0.64 0.64 0.64],'LineWidth',1.5)
or1 = [traj1(end,1)-a*sin(traj1(end,3)) traj1(end,2)+a*cos(traj1(end,3)) ];
or2 = [traj1(end,1)-(a+L/5)*sin(traj1(end,3)) traj1(end,2)+(a+L/5)*cos(traj1(end,3))];
ord = (or2 - or1)/2;
quiver(or1(1),or1(2),ord(1),ord(2),0,'r','LineWidth',3,'MaxHeadSize',10)

axis equal tight
axis([x_min x_max 0 z_max])
box on
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex')
xlabel('$kx$','interpreter','latex')
ylabel('$kz$','interpreter','latex')
xticks([0 pi 2*pi])
xticklabels({'0','$\pi$','$2\pi$'})
title('Open-loop','interpreter','latex')


nexttile;

%flow 
u = @(t,x) -umax*sign(sin(k*x-w*t));
Ux = wall_x(X,Z,u(T,traj2(end,1)),T);
Uz = wall_z(X,Z,u(T,traj2(end,1)),T);
c = ceil(99*(u(T,traj2(end,1))/umax+1)/2)+1;
even_stream_arrow(X, Z, Ux, Uz, 1, 2, ...
    'LineStyle', '-', 'LineWidth', 0.5, 'Color', [0 0.77 0.93], 'ArrowLength', 4, ...
    'ArrowTipAngle', 20, 'ArrowBaseAngle', 60, 'ArrowDensity', 1);
alpha(.4)
hold on

% trajectory
plot(traj2(1:end,1),traj2(1:end,2),'k','LineWidth',2)
hold on

% swimmer, orientation
swimmer = nsidedpoly(30,'Center',[traj2(end,1) traj2(end,2)],'Radius',a);
plot(swimmer,'FaceAlpha',1,'FaceColor',[0.64 0.64 0.64],'LineWidth',1.5)
or1 = [traj2(end,1)-a*sin(traj2(end,3)) traj2(end,2)+a*cos(traj2(end,3)) ];
or2 = [traj2(end,1)-(a+L/5)*sin(traj2(end,3)) traj2(end,2)+(a+L/5)*cos(traj2(end,3))];
ord = (or2 - or1)/2;
quiver(or1(1),or1(2),ord(1),ord(2),0,'r','LineWidth',3,'MaxHeadSize',10)

axis equal tight
axis([x_min x_max 0 z_max])
box on
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex')
xlabel('$kx$','interpreter','latex')
ylabel('$kz$','interpreter','latex')
xticks([0 pi 2*pi])
xticklabels({'0','$\pi$','$2\pi$'})
title('Feedback','interpreter','latex')


nexttile;

%flow 
u = @(t,x) -umax*sign(sin(k*x-w*t));
Ux = wall_x(X,Z,u(T,traj4(end,1)),T);
Uz = wall_z(X,Z,u(T,traj4(end,1)),T);
c = ceil(99*(u(T,traj4(end,1))/umax+1)/2)+1;
even_stream_arrow(X, Z, Ux, Uz, 1, 2, ...
    'LineStyle', '-', 'LineWidth', 0.5, 'Color', [0 0.77 0.93], 'ArrowLength', 4, ...
    'ArrowTipAngle', 20, 'ArrowBaseAngle', 60, 'ArrowDensity', 1);
alpha(.4)
hold on

% trajectory
plot(traj4(1:end,1),traj4(1:end,2),'k','LineWidth',2)
hold on

% swimmer, orientation
swimmer = nsidedpoly(30,'Center',[traj4(end,1) traj4(end,2)],'Radius',a);
plot(swimmer,'FaceAlpha',1,'FaceColor',[0.64 0.64 0.64],'LineWidth',1.5)
or1 = [traj4(end,1)-a*sin(traj4(end,3)) traj4(end,2)+a*cos(traj4(end,3)) ];
or2 = [traj4(end,1)-(a+L/5)*sin(traj4(end,3)) traj4(end,2)+(a+L/5)*cos(traj4(end,3))];
ord = (or2 - or1)/2;
quiver(or1(1),or1(2),ord(1),ord(2),0,'r','LineWidth',3,'MaxHeadSize',10)

axis equal tight
axis([x_min x_max 0 z_max])
box on
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex')
xlabel('$kx$','interpreter','latex')
ylabel('$kz$','interpreter','latex')
xticks([0 pi 2*pi])
xticklabels({'0','$\pi$','$2\pi$'})
title('$B_2 = -20$','interpreter','latex')


nexttile;

%flow 
u = @(t,x) -umax*sign(sin(k*x-w*t));
Ux = wall_x(X,Z,u(T,traj5(end,1)),T);
Uz = wall_z(X,Z,u(T,traj5(end,1)),T);
c = ceil(99*(u(T,traj5(end,1))/umax+1)/2)+1;
even_stream_arrow(X, Z, Ux, Uz, 1, 2, ...
    'LineStyle', '-', 'LineWidth', 0.5, 'Color', [0 0.77 0.93], 'ArrowLength', 4, ...
    'ArrowTipAngle', 20, 'ArrowBaseAngle', 60, 'ArrowDensity', 1);
alpha(.4)
hold on

% trajectory
plot(traj5(1:end,1),traj5(1:end,2),'k','LineWidth',2)
hold on

% swimmer, orientation
swimmer = nsidedpoly(30,'Center',[traj5(end,1) traj5(end,2)],'Radius',a);
plot(swimmer,'FaceAlpha',1,'FaceColor',[0.64 0.64 0.64],'LineWidth',1.5)
or1 = [traj5(end,1)-a*sin(traj5(end,3)) traj5(end,2)+a*cos(traj5(end,3)) ];
or2 = [traj5(end,1)-(a+L/5)*sin(traj5(end,3)) traj5(end,2)+(a+L/5)*cos(traj5(end,3))];
ord = (or2 - or1)/2;
quiver(or1(1),or1(2),ord(1),ord(2),0,'r','LineWidth',3,'MaxHeadSize',10)

axis equal tight
axis([x_min x_max 0 z_max])
box on
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex')
xlabel('$kx$','interpreter','latex')
ylabel('$kz$','interpreter','latex')
xticks([0 pi 2*pi])
xticklabels({'0','$\pi$','$2\pi$'})
title('$B_2 = -100$','interpreter','latex')

nexttile;

%flow 
u = @(t,x) -umax*sign(sin(k*x-w*t));
Ux = wall_x(X,Z,u(T,traj3(end,1)),T);
Uz = wall_z(X,Z,u(T,traj3(end,1)),T);
c = ceil(99*(u(T,traj3(end,1))/umax+1)/2)+1;
even_stream_arrow(X, Z, Ux, Uz, 1, 2, ...
    'LineStyle', '-', 'LineWidth', 0.5, 'Color', [0 0.77 0.93], 'ArrowLength', 4, ...
    'ArrowTipAngle', 20, 'ArrowBaseAngle', 60, 'ArrowDensity', 1);
alpha(.4)
hold on

% trajectory
plot(traj3(1:end,1),traj3(1:end,2),'k','LineWidth',2)
hold on

% swimmer, orientation
swimmer = nsidedpoly(30,'Center',[traj3(end,1) traj3(end,2)],'Radius',a);
plot(swimmer,'FaceAlpha',1,'FaceColor',[0.64 0.64 0.64],'LineWidth',1.5)
or1 = [traj3(end,1)-a*sin(traj3(end,3)) traj3(end,2)+a*cos(traj3(end,3)) ];
or2 = [traj3(end,1)-(a+L/5)*sin(traj3(end,3)) traj3(end,2)+(a+L/5)*cos(traj3(end,3))];
ord = (or2 - or1)/2;
quiver(or1(1),or1(2),ord(1),ord(2),0,'r','LineWidth',3,'MaxHeadSize',10)

axis equal tight
axis([x_min x_max 0 z_max])
box on
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex')
xlabel('$kx$','interpreter','latex')
ylabel('$kz$','interpreter','latex')
xticks([0 pi 2*pi])
xticklabels({'0','$\pi$','$2\pi$'})
title('$B_2 = 20$','interpreter','latex')

pl.TileSpacing = 'compact';
pl.Padding = 'none';











