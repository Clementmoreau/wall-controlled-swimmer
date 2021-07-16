% optim_wall_time(init_cond, target, k, w, v,...
%    guess_time_control,number_of_control_intervals)
% number of dsicretisation intervals

%addpath('/Users/moreau/Documents/MATLAB/casadi-osx-matlabR2015a-v3.5.5');
%import casadi.*

N = 200;

% parameters
k = 1;
w = [3:15];
v = 1;
umax = 5;

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



% initial guess
T_g = 5;
u_g = -9;

%% solve minimal time problems

p=length(w);
xx=zeros(p,N+1);
zz=zeros(p,N+1);
thth=zeros(p,N+1);
u1=zeros(p,N);
T1=zeros(p,1);
Tswitch=zeros(p,30);
Nswitch=zeros(p,1);

for j=1:p
[xx(j,:), zz(j,:), thth(j,:), u1(j,:), T1(j)] = optim_wall_time([x0,z0,th0],[xf,zf],k, w(j), v, [T_g, u_g],N,A0,A2,B2,a,umax);

% detect switchings
dt1 = T1(j)/N;
du1=u1(j,2:end)-u1(j,1:end-1);

t_switch=[];

for i=1:length(du1)
    if abs(du1(i))>umax
        t_switch=[t_switch dt1*i];
    end
end

Nswitch(j)=length(t_switch);
Tswitch(j,1:Nswitch(j))=t_switch;

end


%% solve minimal time problems for half V0

umax = 10;

p=length(w);
xx=zeros(p,N+1);
zz=zeros(p,N+1);
thth=zeros(p,N+1);
u1=zeros(p,N);
T1=zeros(p,1);
Tswitch2=zeros(p,30);
Nswitch2=zeros(p,1);

for j=1:p
[xx(j,:), zz(j,:), thth(j,:), u1(j,:), T1(j)] = optim_wall_time([x0,z0,th0],[xf,zf],k, w(j), v, [T_g, u_g],N,A0,A2,B2,a,umax);

% detect switchings
dt1 = T1(j)/N;
du1=u1(j,2:end)-u1(j,1:end-1);

t_switch=[];

for i=1:length(du1)
    if abs(du1(i))>umax
        t_switch=[t_switch dt1*i];
    end
end

Nswitch2(j)=length(t_switch);
Tswitch2(j,1:Nswitch2(j))=t_switch;

end

%% graphics

% compute switch period
T_switch_1=zeros(p,1);
for i=1:p
T_switch_1(i)=sum((Tswitch(i,2:Nswitch(i))-Tswitch(i,1:Nswitch(i)-1)))/(Nswitch(i)-1);
end

T_switch_2=zeros(p,1);
for i=1:p
T_switch_2(i)=sum((Tswitch2(i,2:Nswitch2(i))-Tswitch2(i,1:Nswitch2(i)-1)))/(Nswitch2(i)-1);
end

% model
V0 = abs(5*exp(-k*z0)*(1-k*z0));
ww=0:1:15;
T_switch_model_1 = pi./(ww-(v+2*V0/pi));
T_switch_model_2 = pi./(ww-(v+4*V0/pi));

figure(3);clf;

hold on
plot(ww,T_switch_model_1.^-1,'-','Linewidth',1,'Color',[0.64 0.64 0.64])
plot(ww,T_switch_model_2.^-1,'--','Linewidth',1,'Color',[0.64 0.64 0.64])

plot(w,T_switch_1.^-1,'o','Linewidth',1,'Color',[0 0.4470 0.7410],'markerfacecolor',[0 0.4470 0.7410])
plot(w,T_switch_2.^-1,'o','Linewidth',1,'Color',[0.8500 0.3250 0.0980],'markerfacecolor',[0.8500 0.3250 0.0980])

legend({'$\!\!\!\!$','$\!\!\!\!$','$V_0=0.34$','$V_0=0.68$',''},'Interpreter','latex','Location','northwest','NumColumns',2);
axis([0 15 0 5])
box on
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex')
set(gca,'TickLength',[1 1]*2e-2)
xlabel('$\omega$','interpreter','latex')
ylabel('$T_{\mathrm{switch}}^{-1}$','interpreter','latex')

exportgraphics(gcf,'figx_panel3.eps','ContentType','vector')


