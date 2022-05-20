function cte = quadrupleLoadParameters()
%% Description
% This function gerenated a struct with constants of the model dynamics.
%% Load identification parameters
% System order
cte.n = 4;
cte.m = 2;
% Identification data
data = load("identification/A.mat",'A');
cte.A = data.A;
data = load("identification/a_hole.mat",'a');
cte.a = data.a;
data = load("identification/dh_dr.mat",'dh_dr');
cte.dh_dr = data.dh_dr;
data = load("identification/g.mat",'g');
cte.g = data.g;
data = load("identification/gamma.mat",'gamma');
cte.gamma = data.gamma;
data = load("identification/h0.mat",'h0');
cte.h0 = data.h0;
data = load("identification/pump.mat",'c','u_min','u_max','k');
cte.c = data.c;
cte.uMax = data.u_max;
cte.uMin = data.u_min;
cte.k = data.k;

%% Simulation noise parameters  and disturbance
% Continuous-time process noise covariance matrix
cte.Q = 0.05*1e-2*[3.331 0 1.938 0.989;
                    0 3.969 1.717 1.984;
                    1.938 1.717 2.779 0;
                    0.989 1.984 0 4.034];
% Sensor noise covariance matrix
cte.R = 0.001*eye(4);
% Define tanks to which disturbance flows are connected
cte.d_tanks = [1;2];
% Define disturbance scale factor
cte.d_scale = 0.05;

%% Equilibrium level matrices
x = sym('x',[1 cte.n],'real');  % vector of positive water levels 
assume(x,'positive');
u = sym('u',[1,cte.m],'real');  % vector of positive pump actuations 
assume(u,'positive');
% vector of equations for null derivative of the water level
xdot = sym(zeros(cte.n,1));    
for i = 1:cte.n/2
   j = i+cte.n/2;
   xdot(i) = -(cte.a(i)/cte.A(i))*sqrt(2*cte.g)*x(i)+(cte.a(j)/cte.A(i))...
       *sqrt(2*cte.g*x(j))+cte.gamma(i)*cte.k(i)*u(i)/cte.A(i) == 0;
end
i = i+1;
j = cte.n/2;
xdot(cte.n/2+1) = -(cte.a(i)/cte.A(i))*sqrt(2*cte.g*x(i))+...
       (1-cte.gamma(j))*cte.k(j)*u(j)/cte.A(i) == 0;
for i = cte.n/2+2:cte.n
   j = i-cte.n/2-1; 
   xdot(i) = -(cte.a(i)/cte.A(i))*sqrt(2*cte.g*x(i))+...
       (1-cte.gamma(j))*cte.k(j)*u(j)/cte.A(i) == 0;
end
% Solve xdot = 0 for equilibrium solution
% Supress warning concerning vality of soltions according to
% assunptions made in the variables
warning('off')
sol = solve(xdot,[x(cte.n/2+1:end) u(1:end)]);
warning('on')
sol = struct2cell(sol);
% Plugin respective cooefficients in the entries of alpha and beta
alpha = zeros(cte.n/2,cte.n/2+nchoosek(cte.n/2,2));
for i = cte.n/2+1:cte.n
    [alpha(i-cte.n/2,:),~] = coeffs(sol{i-cte.n/2,1},x(1:cte.n/2));
end
beta = zeros(cte.m,cte.n/2);
for i = cte.n+1:3*cte.n/2
    [beta(i-cte.n,:),~] = coeffs(sol{i-cte.n/2,1},x(1:cte.n/2));
end
cte.alpha = alpha;
cte.beta = beta;
% Discretization step, i.e., CLK period
cte.dT = 1; %(s)
end
