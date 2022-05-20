%% LQR control of the quadruple-tank process
% The goal is to track piece-wise constant signals with the level of the
% lower tanks
clear;

%% 1. Linearized discrete dynamics
% Linearization of the quadruple tank process about an operation point
% Operation point: 
% Equilibrium point corresponding to h1 = 14 cm; h2 = 14 cm
% Load quadruple-tank process parameters  
cte = quadrupleLoadParameters();
% Set dimensions of the system
n = cte.n;
m = cte.m;
dT = cte.dT;
% Compute equilibrium level and actuation such that h1 = 14 cm; h2 = 14 cm
[xEq,uEq] = computeEquilibriumLevels([14;14],cte);
% Compute linearized entries of matrices A and B for a continuous process
[A_cont,B_cont] = Dxdot(xEq,cte);
% Discretize dynamics with sampling interval dT
G = expm([A_cont B_cont; zeros(m,n) zeros(m,m)]*dT);
A = G(1:n,1:n);
B = G(1:n,n+1:end);
C = eye(cte.n);
% Poles of open loop system
fprintf("1. Open-loop system poles:\n");
ddamp(A,dT)
%% 2. Check controllability
fprintf("-------------------------\n");
fprintf("2. Check controllability:\n");
control_mat = ctrb(A,B);
fprintf("Rank of ctrb: %d | is controllable: %d \n",...
    rank(control_mat),rank(control_mat)==n);
%% 3. Check observability
fprintf("-------------------------\n");
fprintf("3. Check observability:\n");
observ_mat = obsv(A,C);
fprintf("Rank of obsv: %d | is observable: %d \n",...
    rank(observ_mat),rank(observ_mat)==n);
%% 5. Compute vector of gains K
% Assuming that full-state feedback is available
% Note that, in this example, illustrative gains are computed
% To ensure good performance this selection should be thoughtfull
fprintf("-------------------------\n");
fprintf("5. Compute LQR gain matrix of gains K\n");
% Weight matrices
R = eye(m); % Must be a positive scalar
Q = diag([1/(12^2),1/(12^2),0,0]);  
% For zero tracking error and conatnt disurbance rejection add integral 
% Tracking output
H = [eye(2) zeros(2,2)];
% Augmented state
A_ = [A zeros(n,n/2); H eye(2)];
B_ = [B; zeros(2,2)];
Q_ = 100*blkdiag(Q,0.05*Q(1:2,1:2));
R_ = R;
% Compute lqr gain
[K,~,~] = dlqr(A_,B_,Q_,R_);
fprintf("Controlled system poles with integral action:\n");
ddamp(A_-B_*K)
% Feedforward gain
N = inv(H*((eye(4)-(A-B*K(:,1:4)))\B));

%% Auxiliary functions
%% Dxdot - Description 
% This function computes matrices A and B of the continuous linearized 
% model given an equilibrium state.
% Input:    - x_eq: equilibrium state vector
%           - cte: struct with constants of the model dynamics
function [A,B] = Dxdot(x_eq,cte)
% Initialize matrices
A = zeros(cte.n,cte.n);
B = zeros(cte.n,cte.m);
% Compute time constants of each tank
T = zeros(cte.n,1);
for i = 1:cte.n
   T(i) = (cte.A(i)/cte.a(i))*sqrt(2*x_eq(i)/cte.g);
end
% --- Compute A ---
% Lower tanks
for i = 1:cte.n
    if abs(T(i)) > eps 
        A(i,i) = -1/T(i);
    else
        A(i,i) = 0;
    end
end
% Upper tanks
for i = 1:cte.n/2
    j = i+cte.n/2;
    if abs(T(j)) > eps 
        A(i,j) = cte.A(j)/(cte.A(i)*T(j));
    else
        A(i,j) = 0;
    end
end
% --- Compute B ---
% Lower tanks
for i = 1:cte.n/2
    B(i,i) = cte.gamma(i)*cte.k(i)/cte.A(i);
end
% The pump of the last lower tanks feeds the first upper tank
B(cte.n/2+1,cte.n/2) = (1-cte.gamma(cte.n/2))*cte.k(cte.n/2)/cte.A(cte.n/2+1);
% Upper tanks except for the first
for i = cte.n/2+2:cte.n
    j = i-1-cte.n/2;
    B(i,j) =  (1-cte.gamma(j))*cte.k(j)/cte.A(i);
end
end

%% computeEquilibriumLevels - Description
% This function computes the equilibrium levels corresponding to a given
% reference to the lower tanks, according to [1]
% Input :   - ref: reference vector to the lower tanks
%           - cte: parameters of the network 
% Output:   - xEq: vector of equilibrium water levels
%           - uEq: vector of equilibrium pump actuations
function [xEq,uEq] = computeEquilibriumLevels(ref,cte)
    % vector of reference levels and respective square root combinations
    xauxalpha = zeros(cte.n/2+nchoosek(cte.n/2,2),1);
    count = 1;
    for i = 1:cte.n/2
        for j = i:cte.n/2
            xauxalpha(count) = sqrt(ref(i))*sqrt(ref(j)); 
            count = count+1;
        end
    end
    % vector of the square root of the reference levels 
    xauxbeta = zeros(cte.n/2,1);
    for i = 1:cte.n/2
        xauxbeta(i) = sqrt(ref(i)); 
    end
    % Compute equilibrium levels and actuation
    xEq = zeros(cte.n,1);
    xEq(1:cte.n/2) = ref;
    xEq(cte.n/2+1:end) = cte.alpha*xauxalpha;
    uEq = cte.beta*xauxbeta;
end

