% Micro-swimmer controlled by shear
%
% For more information see: http://labs.casadi.org/OCP
function [x, z, th, u, T] = optim_wall_time(init_cond, target, k, w, v,...
    guess_time_control,number_of_control_intervals,A0,A2,B2,a,umax)

N = number_of_control_intervals; % number of control intervals

opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
Y = opti.variable(3,N+1); % state trajectory
x = Y(1,:);
z = Y(2,:);
th = Y(3,:);
u = opti.variable(1,N);   % control trajectory
T = opti.variable();      % final time

dt = T/N; % length of a control interval

% ---- objective          ---------
 opti.minimize(T); % minimal time

% ---- dynamic constraints --------
swim = @(Y,u,t) [-v*sin(Y(3));...
                 v*cos(Y(3));...
                 0];
    
squirm = @(Y,u,t) [3*a*a*sin(2*Y(3))*(A2-B2)/5/8/(max(Y(2),0.001))^2;
                   -3*a*a*(A0+(1+3*cos(2*Y(3)))*(A2-B2)/5)/16/(max(Y(2),0.001))^2;
                   3*a*a*sin(2*Y(3))*(A2-B2)/5/16/(max(Y(2),0.001))^3];
      
wall = @(Y,u,t) [u*exp(-k*Y(2))*(1-k*Y(2))*sin(k*Y(1)-w*t);
                 -u*exp(-k*Y(2))*k*Y(2)*cos(k*Y(1)-w*t);
                 u*exp(-k*Y(2))*k*sin(k*Y(1)-w*t)/2];
             
f = @(Y,u,t) swim(Y,u,t) + squirm(Y,u,t) + wall(Y,u,t); % dz/dt = f(z,u)

for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(Y(:,k),         u(:,k),(k-1)*dt);
   k2 = f(Y(:,k)+dt/2*k1, u(:,k),(k-1/2)*dt);
   k3 = f(Y(:,k)+dt/2*k2, u(:,k),(k-1/2)*dt);
   k4 = f(Y(:,k)+dt*k3,   u(:,k),k*dt);
   Y_next = Y(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(Y(:,k+1)==Y_next); % close the gaps
end

% ---- constraints -----------
opti.subject_to(-umax<=u<=umax);           % control is limited
opti.subject_to(z>1);

% ---- boundary conditions --------
opti.subject_to(x(1)==init_cond(1)); % initial condition
opti.subject_to(z(1)==init_cond(2)); 
opti.subject_to(th(1)==init_cond(3));
opti.subject_to(x(N+1)==target(1)); % target position
opti.subject_to(z(N+1)==target(2));

% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive

% ---- initial values for solver ---
opti.set_initial(T, guess_time_control(1));
opti.set_initial(u, guess_time_control(2));

% ---- solve NLP              ------
opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

% output
x=sol.value(x);
z=sol.value(z);
th=sol.value(th);
u=sol.value(u);
T=sol.value(T);

end



