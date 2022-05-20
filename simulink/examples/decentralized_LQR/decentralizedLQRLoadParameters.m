function cte = decentralizedLQRLoadParameters()
%% Description
% This function gerenated a struct with constants of the model dynamics and
% controller paramenters.
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

%% Discretization
cte.dTlin = 10;
cte.dT = 1;

%% Equilibrium matrices 
data = load('EqMatrices.mat','alpha','beta');
cte.alpha = data.alpha;
cte.beta = data.beta;
% Discretization step, i.e., CLK period

%% Define LQT algorithm parameters 
% Extension to infinite-horizon paramenters
cte.d = 10;
cte.T = 30;
cte.iLQRIt = 50;
cte.iLQReps = 1e-4;
% Anti-windup
cte.AntiWU = 10;
% LQR weights
cte.R_LQR = eye(2);
cte.h = eye(2,4);
cte.H = zeros(4,6);
cte.H(1:2,1:2) = eye(2);
cte.H(3:end,5:end) = eye(2);
cte.Qt = zeros(4);
cte.Qi = 0.05*eye(2);
cte.Qt(1:2,1:2) = 20*eye(2);
cte.Qt(3:4,3:4) = 0.05*eye(2);
% When only a controller is used, the lower tanks only known their own
% water-level
cte.E = [1 0;
         0 1;
         0 0;
         0 0;
         1 0;
         0 1]';     
end



