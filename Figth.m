% optim_wall_time(init_cond, target, k, w, v,...
%    guess_time_control,number_of_control_intervals)
% number of dsicretisation intervals

%addpath('/Users/moreau/Documents/MATLAB/casadi-osx-matlabR2015a-v3.5.5');
%import casadi.*

N = 200;

% parameters
k = 1;
w = 12;
v = 1;
umax = 10;

L = 2*pi/k;
tau = 2*pi./w;

% initial condition
x0 = 0;
z0 = 3;
th0 = [-pi/4,-pi,-pi/2];

% target
thf = [-3*pi/4,0,pi/2];

%radius
a = 0.01;

% squirmer values
A0=0;
A2=0;
B2=0;

% initial guess
T_g = 6;
u_g = -9;

xx=zeros(3,N+1);
zz=zeros(3,N+1);
thth=zeros(3,N+1);
u1=zeros(3,N);
T1=zeros(3,1);


% solve minimal time problem

for j=1:3
    
    [xx(j,:), zz(j,:), thth(j,:), u1(j,:), T1(j)] = optim_wall_theta([x0,z0,th0(j)],thf(j),k, w, v, [T_g, u_g],N,A0,A2,B2,a,umax);
    
end

%% Visualization

wall_x = @(x,z,u,t) u*exp(-k*z).*(1-k*z).*sin(k*x-w*t);
wall_z = @(x,z,u,t) -u*k*exp(-k*z).*z.*cos(k*x-w*t);
wall_th = @(x,z,u,t) u*k*exp(-k*z).*sin(k*x-w*t)/2;

figure(1);clf

pl=tiledlayout(3,3)

for j=1:3
    
    % trajectory
    nexttile;
    z_max = 4/k;
    dz = z_max/20;
    x_min = -0.5;%(min(xx(j,:)) - L/4);
    x_max = 5;%(max(xx(j,:)) + L/4);
    dx = (x_max-x_min)/30;
    [X,Z] = meshgrid(x_min:dx:x_max,0:dz:z_max);
    
    
    col = colormap(jet(105));
    dt1 = T1(j)/N;
    
        t = dt1*N;
        hold off
        
        % flow field
        Ux = wall_x(X,Z,u1(j,end),t);
        Uz = wall_z(X,Z,u1(j,end),t);
        c = ceil(99*(u1(j,end)/umax+1)/2)+1;
        even_stream_arrow(X, Z, Ux, Uz, 1, 2, ...
        'LineStyle', '-', 'LineWidth', 0.5, 'Color', [0 0.77 0.93], 'ArrowLength', 4, ...
        'ArrowTipAngle', 20, 'ArrowBaseAngle', 60, 'ArrowDensity', 0.5);
        alpha(.4)
        hold on
    
        
        % trajectory
        plot(xx(j,1:end),zz(j,1:end),'k','LineWidth',2)
        
        
        % swimmer, orientation at beginning
        swimmer = nsidedpoly(20,'Center',[xx(j,1) zz(j,1)],'Radius',a);
        plot(swimmer)
        or1 = [xx(j,1)-a*sin(thth(i)) zz(j,1)+a*cos(thth(j,1)) ];
        or2 = [xx(j,1)-(a+L/7)*sin(thth(j,1)) zz(j,1)+(a+L/7)*cos(thth(j,1))];
        ord = or2 - or1;
        quiver(or1(1),or1(2),ord(1),ord(2),0,'r','LineWidth',3,'MaxHeadSize',3)
        %final orientation
        swimmer = nsidedpoly(20,'Center',[xx(j,end) zz(j,end)],'Radius',a);
        plot(swimmer)
        or1 = [xx(j,end)-a*sin(thth(j,end)) zz(j,end)+a*cos(thth(j,end)) ];
        or2 = [xx(j,end)-(a+L/7)*sin(thth(j,end)) zz(j,end)+(a+L/7)*cos(thth(j,end))];
        ord = or2 - or1;
        quiver(or1(1),or1(2),ord(1),ord(2),0,'r','LineWidth',3,'MaxHeadSize',3)
        
        % title
        % title([],'interpreter','latex')
        
        axis equal tight
        axis([x_min x_max 0 z_max])
        box on
        set(gca,'FontSize',18)
        set(gca,'TickLabelInterpreter','latex')
        % set(gca,'TickLength',[1 1]*2e-2)
        %xlabel('$kx$','interpreter','latex')
        ylabel('$kz$','interpreter','latex')
        % colorbar('on')
        
        drawnow
        
        %orientation
        nexttile;
        tt=0:dt1:T1(j);
        plot(tt,thth(j,:))
        %axis tight
        grid on
        box on
        set(gca,'FontSize',18)
        set(gca,'TickLabelInterpreter','latex')
        % xlabel('$t$','interpreter','latex')
        ylabel('$\theta$','interpreter','latex')
        % colorbar('on')

        % local flow curl
        nexttile;
        clear w_y
        w_y=zeros(1,N);
        for i=1:N
            w_y(i)=wall_th(xx(j,i),zz(j,i),u1(j,i),tt(i));
        end
        plot(tt(1:end-1),w_y)
        %axis tight
        grid on
        box on
        set(gca,'FontSize',18)
        set(gca,'TickLabelInterpreter','latex')
        % set(gca,'TickLength',[1 1]*2e-2)
        % xlabel('$t$','interpreter','latex')
        ylabel('$\Omega_y$','interpreter','latex')
        % colorbar('on')
        
        
end

nexttile(7)
xlabel('$kx$','interpreter','latex')
nexttile(8)
xlabel('$t$','interpreter','latex')
nexttile(9)
xlabel('$t$','interpreter','latex')

    
pl.TileSpacing = 'none';
pl.Padding = 'none';

exportgraphics(gcf,'figx_th.eps','ContentType','vector')
    
    
    
    
    
    
    
    
    
    
    
