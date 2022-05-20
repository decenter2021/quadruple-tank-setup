%% Description
% Matlab system that simulates the quadruple-tank setup.

%% Define Matlab system
classdef quadrupleSimObj < matlab.System & ...
     matlab.system.mixin.Propagates
    % quadrupleSimObj Simulates the dynamics of the quadruple-tank setup.
    properties(Nontunable)
    end
    % Define system states
    properties(DiscreteState) 
        x;
    end
    % Pre-computed constants
    properties(Access = private)
        cte;
    end 
    methods(Access = protected)
        
        function resetImpl(~)
            % Initialize / reset discrete-state properties      
        end

        function [out,out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [4 1];
            out2 = out;
        end
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            % Load quadruple-tank process parameters
            obj.cte = quadrupleLoadParameters();
            % Initialize state 
            % Non-zero uinitialization for numeric stability 
            % (e.g. when simulating just one tank)
            obj.x = ones(4,1);
        end

        function [y_gt,y] = stepImpl(obj,u,d) 
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            % Compute the equilibrium levels corresponding to the current water level 
            % of the lower tanks, according to [1]
            [xEq,~] = computeEquilibriumLevels(obj.x(1:obj.cte.n/2,1),obj.cte);
            % Compute linearized entries of matrix A for a continuous process
            [A_cont,~] = Dxdot(xEq,obj.cte);
            % Discretize the process noise covariance matrix
            G = expm(obj.cte.dT*[-A_cont obj.cte.Q ; zeros(obj.cte.n) A_cont']);
            Qprocess = transpose(G(obj.cte.n+1:end,obj.cte.n+1:end))*G(1:obj.cte.n,obj.cte.n+1:end);
            % Ensure symmetry for numeric stability
            Qprocess = 0.5*(Qprocess+Qprocess');
            % Simulate nonlinear system 
            [~,nonLinSol] = ode45(@(t,x) xdotContinuous(x,u,d,obj.cte),[0 obj.cte.dT],obj.x);
            % Add process noise
            obj.x = nonLinSol(end,:)'+transpose(mvnrnd(zeros(obj.cte.n,1),Qprocess));
            obj.x(obj.x<0) = 0;  %For numeric stability
            % Add sensor noise
            y_gt = obj.x;
            y = obj.x + transpose(mvnrnd(zeros(obj.cte.n,1),obj.cte.R));
            y(y<=0) = 0; %For numeric stability
        end
        
        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj,name)
            if strcmp(name,'x')
                sz = [4 1];
                dt = 'double';
                cp = false;
            else
                error(['Error: Incorrect State Name: ', name.']);
            end
        end    
        function varargout = getOutputDataTypeImpl(obj)
             varargout = cell(1, getNumOutputs(obj));
            for i = 1:getNumOutputs(obj)
                varargout{i} = 'double';
            end
        end
        function varargout = isOutputComplexImpl(obj)
            varargout = cell(1, getNumOutputs(obj));
            for i = 1:getNumOutputs(obj)
                varargout{i} = false;
            end            
        end
        function varargout = isOutputFixedSizeImpl(obj)
            % Get outputs fixed size.
            varargout = cell(1, getNumOutputs(obj));
            for i = 1:getNumOutputs(obj)
                varargout{i} = true;
            end            
        end
    end
end

%% Auxiliary functions
%% xdotContinuous - Description
% This function computes the derivative of the state vector using the non 
% linear dynamics of the model, for a given state and control vector.
% Input:    - x: state vector 
%           - u: actuation vector
% Output:   - xdot: derivative of the state vector
function xdot = xdotContinuous(x,u,d,cte)
% Initialize xdot vector
xdot = zeros(cte.n,1);
% Compute xdot acconding to the non linear model
% For thr lower tanks
for i = 1:cte.n/2
   j = i+cte.n/2;
   xdot(i) = -(cte.a(i)/cte.A(i))*sqrt(2*cte.g*x(i))+(cte.a(j)/cte.A(i))*sqrt(2*cte.g*x(j))+...
       cte.gamma(i)*cte.k(i)*u(i,1)/cte.A(i);
end
% The first upper tank is connected to the same pump as the last lower tank
i = cte.n/2+1;
j = cte.n/2;
xdot(cte.n/2+1) = -(cte.a(i)/cte.A(i))*sqrt(2*cte.g*x(i))+...
       (1-cte.gamma(j))*cte.k(j)*u(j)/cte.A(i);
% For the upper tanks except for the first
for i = cte.n/2+2:cte.n
   j = i-cte.n/2-1; 
   xdot(i) = -(cte.a(i)/cte.A(i))*sqrt(2*cte.g*x(i))+(1-cte.gamma(j))*cte.k(j)*u(j)/cte.A(i);
end 
% Include disturbance simulation
for i = 1:2
    if d(i)>max(cte.c(3,1),cte.c(2,1))
        q_d = cte.k(1)*cte.c(1,1)*cte.d_scale*(d(i)-cte.c(3,1))/(d(i)-cte.c(2,1));
        xdot(cte.d_tanks(i),1) = xdot(cte.d_tanks(i),1) + q_d/cte.A(cte.d_tanks(i));
    end
end
% For numeric stability (e.g. when simulating just one tank)
for i = 1:4
    if x(i)<1e-2
       xdot(i) = 0; 
    end
end


end

%% Dxdot - Description 
% This function computes matrices A and B of the continuous linearized 
% model given an equilibrium state.
% Input:    - x_eq: equilibrium state vector
%           - cte: struct with constants of the model dynamics
function [A,B] = Dxdot(x_eq,cte)
% Initialize matrices
A = zeros(cte.n,cte.n);
B = zeros(cte.n,cte.m);
% For numeric stability (e.g. when simulating just one tank)
x_eq(x_eq<0) = 0;
% Compute time constants of each tank
T = zeros(cte.n,1);
for i = 1:cte.n
   T(i) = (cte.A(i)/cte.a(i))*sqrt(2*x_eq(i)/cte.g);
end
% For numeric stability (e.g. when simulating just one tank)
T(T<0.1) = 0.1;
% --- Compute A ---
% Lower tanks
for i = 1:cte.n
    A(i,i) = -1/T(i);
end
% Upper tanks
for i = 1:cte.n/2
    j = i+cte.n/2;
    A(i,j) = cte.A(j)/(cte.A(i)*T(j));
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

%% References
% [1] L. Pedroso, and P. Batista (xxx), Discrete-time decentralized linear 
% quadratic control for linear time-varying systems, Int J Robust Nonlinear
% Control, xxx;xx:x?x. [Submitted to journal]