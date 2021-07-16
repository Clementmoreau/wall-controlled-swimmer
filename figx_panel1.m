% optim_wall_time(init_cond, target, k, w, v,...
%    guess_time_control,number_of_control_intervals)
% number of dsicretisation intervals

%addpath('/Users/moreau/Documents/MATLAB/casadi-osx-matlabR2015a-v3.5.5');
%import casadi.*

N = 200;

% parameters
k = 1;
w = [4,8,12];
v = 1;
umax = 10;

L = 2*pi/k;
tau = 2*pi./w;

% initial condition
x0 = 0;
z0 = 2;
th0 = -pi/2;

% target
xf = x0+L;
zf = 2;

%radius
a = 0.1;

% squirmer values
A0=0;
A2=0;
B2=0;

% initial guess
T_g = 5;
u_g = -9;
% solve minimal time problem 

xx=zeros(3,N+1);
zz=zeros(3,N+1);
thth=zeros(3,N+1);
u1=zeros(3,N);
T1=zeros(3,1);


for j=1:3
[xx(j,:), zz(j,:), thth(j,:), u1(j,:), T1(j)] = optim_wall_time([x0,z0,th0],[xf,zf],k, w(j), v, [T_g, u_g],N,A0,A2,B2,a,umax);

% %% detect switchings
% dt1 = T1/N;
% du1=u1(2:end)-u1(1:end-1);
% 
% t_switch=[];
% 
% for i=1:length(du1)
%     if abs(du1(i))>1.5*umax
%         t_switch=[t_switch dt1*i];
%     end
% end

end

%% Visualization
figure(1);clf
pl=tiledlayout(3,2)

for j=1:3
nexttile;

z_max = 5/k;
dz = z_max/20;
x_min = (min(x0,xf) - L/2);
x_max = (max(x0,xf) + L/2);
dx = (x_max-x_min)/60;
[X,Z] = meshgrid(x_min:dx:x_max,0:dz:z_max);

wall_x = @(x,z,u,t) u*exp(-k*z).*(1-k*z).*sin(k*x-w(j)*t);                 
wall_z = @(x,z,u,t) -u*k*exp(-k*z).*z.*cos(k*x-w(j)*t);

col = colormap(jet(105));

for i = N:N
    dt1 = T1(j)/N;
    t = dt1*i;
    hold off
    
    % flow field
    Ux = wall_x(X,Z,u1(j,i),t);
    Uz = wall_z(X,Z,u1(j,i),t);
    c = ceil(99*(u1(j,i)/umax+1)/2)+1;
    even_stream_arrow(X, Z, Ux, Uz, 1, 4, ...
    'LineStyle', '-', 'LineWidth', 0.5, 'Color', [0 0.77 0.93], 'ArrowLength', 4, ...
    'ArrowTipAngle', 20, 'ArrowBaseAngle', 60, 'ArrowDensity', 0.5);
    alpha(.4)
    hold on
    
    % trajectory
    plot(xx(j,1:i),zz(j,1:i),'k','LineWidth',2)
    
    % swimmer, orientation (and local flow optional)
    swimmer = nsidedpoly(25,'Center',[xx(j,i) zz(j,i)],'Radius',a);
    plot(swimmer,'FaceAlpha',1,'FaceColor',[0.64 0.64 0.64],'LineWidth',1.5)
%     or1 = [x1(i)-a*sin(th1(i)) z1(i)+a*cos(th1(i)) ];
%     or2 = [x1(i)-(a+L/5)*sin(th1(i)) z1(i)+(a+L/5)*cos(th1(i))];
%     ord = or2 - or1;
%     quiver(or1(1),or1(2),ord(1),ord(2),0,'r','LineWidth',2)
    
    % title
    title(['$\omega = ',num2str(w(j)),'$'],'interpreter','latex')
    
    axis equal tight
    axis([x_min x_max 0 z_max])
    box on
    set(gca,'FontSize',20)
    set(gca,'TickLabelInterpreter','latex')
    set(gca,'TickLength',[1 1]*2e-2)
    %xlabel('$kx$','interpreter','latex')
    ylabel('$kz$','interpreter','latex')
    % colorbar('on')
    
    drawnow
end

nexttile;
stairs([1:N]*dt1,u1(j,:),'b','LineWidth',1.5);
% legend(['T = ',num2str(T1)],'Location','northwest')
title(['$T_{\mathrm{min}} = ',num2str(round(T1(j)*100)/100),'$'],'interpreter','latex')
grid on
box on
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex')
set(gca,'TickLength',[1 1]*2e-2)
%xlabel('$t$','interpreter','latex')
ylabel('$u$','interpreter','latex')

end

pl.TileSpacing = 'none';
pl.Padding = 'none';

exportgraphics(gcf,'figx_panel1.eps','ContentType','vector')






