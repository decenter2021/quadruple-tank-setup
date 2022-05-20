classdef controllerObj < matlab.System & ...
     matlab.system.mixin.Propagates
    % Decentralized LQT using the one-step method.
    % Suports code generation.
    properties(Nontunable)
        dT = 1; % Sample time (s)
        T = 30; % Window length
        d = 10; % d
    end
    properties(DiscreteState)
        K;
        u_a;
        x_bar;
        u_bar;
        dCount;
    end
    % Pre-computed constants
    properties(Access = private)
        cte;
    end 
    methods(Access = protected) 
        function resetImpl(~)
            % Initialize / reset discrete-state properties      
        end
        function [out] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [2 1];
        end
        
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.cte = decentralizedLQRLoadParameters();
            obj.dCount = 0;
            obj.K = zeros(2,6*obj.d);
            obj.u_a = zeros(2,obj.d);
            obj.u_bar = zeros(2,obj.d);
            obj.x_bar = zeros(4,obj.d);
        end

        function [u] = stepImpl(obj,WL,xI,x_bar,u_bar) 
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            if rem(obj.dCount,obj.d) == 0
                [obj.K,obj.u_a] = iLQR(WL,x_bar,u_bar,obj.cte,obj.d);
                obj.u_bar = u_bar(:,1:obj.d);
                obj.x_bar = x_bar(:,1:obj.d);
            end
            u = -obj.K(:,6*rem(obj.dCount,obj.d)+1:6*rem(obj.dCount,obj.d)+6)*([WL;xI]-[obj.x_bar(:,rem(obj.dCount,obj.d)+1);zeros(2,1)])+...
                obj.u_bar(:,rem(obj.dCount,obj.d)+1)...
                + obj.u_a(:,rem(obj.dCount,obj.d)+1);
            obj.dCount = obj.dCount + 1;
        end
        
        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj,name)
            if strcmp(name,'x_bar')
                sz = [4 obj.d];
                dt = 'double';
                cp = false;
            elseif strcmp(name,'K')
                sz = [2 6*obj.d];
                dt = 'double';
                cp = false;
            elseif strcmp(name,'u_bar')
                sz = [2 obj.d];
                dt = 'double';
                cp = false;
            elseif strcmp(name,'u_a')
                sz = [2 obj.d];
                dt = 'double';
                cp = false;
            elseif strcmp(name,'dCount')
                sz = [1 1];
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

%% iLQR - Description
% This function computes the iLQR (iterative LQR gains). It is necessary
% because of the nonlinearity of the N tank network. The computation of the
% gains for a finite window using either a centralized or decentralized
% method require that the future dynamics of the system are known. However,
% for this system, the system dynamics in the future depend on the future
% state. iLQR iterative procedure where the future dynamics are being
% updated every time a new MPC window is computed, until convergence. For
% more details see [1].
% Input:    - x0: state at the beginning of the new window
%           - ref: cell array containing x_bar and u_bar
%           - d: number of gains to output out of those computed for the
%           whole window
%           - cte: struct with the necessary constants and parameters
%           - m: number of the method used for the computation of LQR gains
% Output:   - K: struct of LQR gains
%           - u_a: struct of addictional command actions
function [Kout,u_a] = iLQR(x0,x_bar,u_bar,cte,d)
    % Necessary for code generation
    p = cte.T+1;
    q = cte.T;
    % Initialize cell array for the future dynamics of the system 
    system = cell(p,6);
    % --- Forward pass variables ---
    % sequence of states throughout the window
    
    x = cell(1,p);
    % sequence of additional command action throughout the window
    u_a = zeros(2,cte.T); 
    % sequence of command action throughout the window
    uControlDisc = cell(1,q);
    % sequence of gain matrices
    K = cell(q,1);
    % sequence of command action throughout the window (previous iteration)
    % to check when convergence is reached
    prevuControlDisc = cell(1,q);
    % Time of discrete-time intants
    t_disc = 0:cte.dT:cte.T;
    % Code generation init
    for i = 1:p
        x{1,i} = zeros(6,1);
    end
    for i = 1:p
        for j = 1:6
            system{i,j} = 0;
        end
    end
    for i = 1:q
        K{i,1} = zeros(2,6);
        uControlDisc{1,i} = zeros(2,1);
        prevuControlDisc{1,i} = zeros(2,1);
    end
    
    % Perform the iLQR iterations up to a maximum of cte.iLQRIt iterations
    for k = 1:cte.iLQRIt
       % Forward pass
       if k == 1
           % Initial propagated system assumes level is mantained constant
           [system{1,1},system{1,2},system{1,3},system{1,4},system{1,5},system{1,6}]=...
               getDiscreteDynamics(x0,cte);
           for i = 2:cte.T+1
               for p = 1:6
                    system{i,p} = system{1,p};
               end
           end
       else
           for i = 1:cte.T % Simulate forward pass
                if i == 1
                    % Compute the linearized dynamics for the first instant 
                    [system{i,1},system{i,2},system{i,3},system{i,4},system{i,5},system{i,6}]...
                    = getDiscreteDynamics(x0,cte);
                    % (assumes level is mantained constant)
                    for l = i+1:i+cte.dTlin-1
                        for p = 1:6
                             system{l,p} = system{i,p};
                        end
                    end
                    % Measure simulated output
                    x{1,i} = [x0;0;0];
                    % Compute aditional command action
                    u_a(:,i) = (cte.h*system{i,2}(1:cte.n,1:cte.m))\cte.h*...
                       (x_bar(:,min(i+1,size(x_bar,2)))-x_bar(:,min(i,size(x_bar,2))));
                    % Control law [1]: u(k) = -K(k)(x(k)-x_bar(k))+u_bar(k)+u_a(k)
                    uControlDisc{1,i} = -K{i,1}*(x{1,i}-[x_bar(:,min(i,size(x_bar,2)));zeros(cte.n/2,1)])...
                        +u_a(:,i)+u_bar(:,min(i,size(x_bar,2)));
                    % Saturate commands to the pumps
                    uControlDisc{1,i}(uControlDisc{1,i}<cte.uMin(1)) = cte.uMin(1);
                    uControlDisc{1,i}(uControlDisc{1,i}>cte.uMax(1)) = cte.uMax(1);
                    % Simulate nonlinear dynamics
                    [~,nonLinSol] = ode45(@(t,x) xdotContinuous(x,uControlDisc{1,min(floor(t/cte.dT)+1,round(t_disc(i+1)/cte.dT))},cte),[t_disc(i) t_disc(i+1)],x0(1:cte.n,1));
                    %x{1,i+1}(1:cte.n,1) = deval(nonLinSol, t_disc(i+1));
                    x{1,i+1}(1:cte.n,1) = nonLinSol(end);
                    % Integrate tracking error
                    x{1,i+1}(cte.n+1:3*cte.n/2) = x{1,i}(cte.n+1:3*cte.n/2)+x{1,i+1}(1:cte.n/2,1)-x_bar(1:cte.n/2,i+1);
                else
                    % if a new linearization is necessary
                    if rem(i-1,cte.dTlin) == 0 
                       [system{i,1},system{i,2},system{i,3},system{i,4},system{i,5},system{i,6}]...
                       = getDiscreteDynamics(x{1,i},cte);
                       for l = i+1:cte.T+1
                            for p = 1:6
                                system{l,p} = system{i,p};
                            end
                       end
                    end
                    % Compute aditional command action
                    u_a(:,i) = (cte.h*system{i,2}(1:cte.n,1:cte.m))\cte.h*(x_bar(:,min(i+1,size(x_bar,2)))-x_bar(:,min(i,size(x_bar,2))));
                    % Control law [1]: u(k) = -K(k)(x(k)-x_bar(k))+u_bar(k)+u_a(k)
                    uControlDisc{1,i} = -K{i,1}*(x{1,i}-[x_bar(:,min(i,size(x_bar,2)));zeros(cte.n/2,1)])+u_a(:,i)+u_bar(:,min(i,size(x_bar,2)));
                    uControlDisc{1,i}(uControlDisc{1,i}<cte.uMin(1)) = cte.uMin(1);
                    uControlDisc{1,i}(uControlDisc{1,i}>cte.uMax(1)) = cte.uMax(1);
                    % Simulate nonlinear dynamics
                    [~,nonLinSol] = ode45(@(t,x) xdotContinuous(x,uControlDisc{1,min(floor(t/cte.dT)+1,round(t_disc(i+1)/cte.dT))},cte),[t_disc(i) t_disc(i+1)],x{1,i}(1:cte.n,1));
                    %x{1,i+1}(1:cte.n,1) = deval(nonLinSol, t_disc(i+1));
                    x{1,i+1}(1:cte.n,1) = nonLinSol(end);
                    % Integrate tracking error
                    x{1,i+1}(cte.n+1:3*cte.n/2) = x{1,i}(cte.n+1:3*cte.n/2)+x{1,i+1}(1:cte.n/2,1)-x_bar(1:cte.n/2,min(i+1,size(x_bar,2)));
                    % Anti windup for integral action according to [1]
                    for j = 1:cte.n/2
                       if abs(x{1,i}(cte.n+j)) > cte.AntiWU
                           x{1,i}(cte.n+j) = cte.AntiWU*abs(x{1,i}(cte.n+j))/x{1,i}(cte.n+j); 
                       end
                    end
                end
           end
       end
       % --- stopping criterion --- 
       % stop the iteartions if the maximum difference
       % in relation to the actuation computed in the previous iteration
       % falls under cte.iLQReps
       if k > 2
           dif = zeros(1,cte.T);
           for i = 1:cte.T
                dif(1,i) = norm(prevuControlDisc{1,i}-uControlDisc{1,i})/...
                    norm(uControlDisc{1,i});
           end
           if max(dif) < cte.iLQReps 
               break; 
           end
       end
       prevuControlDisc = uControlDisc;
       % --- Compute LQR gains ---
       %if m<= 2 % Centralized
        %    [K,~] = LQRCentralizedLTV(system(:,1:4),cte.T);
       %else % One-step
            [K,~] = LQROneStepLTV(system,cte.E,cte.T);
       %end
       % check if maximum number of iterations was reached and issue
       % warning
       if k == cte.iLQRIt
           fprintf("The maximum number of iLQR iterations was reached before convergence.\n");
       end
    end
    % Output only gains and additional command action that are used
    Kout = zeros(2,6*d);
    for i = 1:d
       Kout(:,(i-1)*6+1:(i-1)*6+6) = K{i,1};
    end
    %K = transpose(K(1:d,1));
    u_a = u_a(:,1:d);
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

%% getDiscreteDynamics - Description
% This function computes the linearized discrete model for a given state.
% The equilibrium state is computed around the level in the lower tanks.
% Input:    - x: state vector
%           - cte: struct of constants of the model dynamics 
% Output:   - linDynamics: 1x7 cell with matrices A,C,Q,R,B, equilibrium
% state vector, and equilibrium control vector, uEq (in this order).
% Note: The state vector in this function includes integral states
function [Alin,Blin,Qlin,Rlin,xEqlin,uEqlin] = getDiscreteDynamics(x,cte)
% Initialize 1x7 cell to store the model matrices and equilibrium vectors
%linDynamics = cell(1,6);
% Compute the equilibrium levels corresponding to the current water level 
% of the lower tanks, according to [1]
[xEq,uEq] = computeEquilibriumLevels(x(1:cte.n/2,1),cte);
% Compute linearized entries of matrices A and B for a continuous process
[A_cont,B_cont] = Dxdot(xEq,cte);
% Discretize dynamics with sampling interval cte.dT
G = expm([A_cont B_cont; zeros(cte.m,cte.n) zeros(cte.m,cte.m)]*cte.dT);
A = G(1:cte.n,1:cte.n);
B = G(1:cte.n,cte.n+1:end);
% Compute matrix A for the system dynamics including integral action, 
% according to [1]
Alin = zeros(3*cte.n/2,3*cte.n/2);
Alin(1:cte.n,1:cte.n) = A;
Alin(cte.n+1:3*cte.n/2,1:cte.n) = cte.h*A;
Alin(1:cte.n,cte.n+1:3*cte.n/2) = zeros(cte.n,cte.n/2);
Alin(cte.n+1:3*cte.n/2,cte.n+1:3*cte.n/2) = eye(cte.n/2);
%linDynamics{1,1} = Alin;
% Compute matrix B for the system dynamics including integral action, 
% according to [1]
Blin = [B;cte.h*B];
% State weigting matrix (icncluding integral action)
Qlin = cte.H'*cte.Qt*cte.H;
% Command action weighting matrix
Rlin = cte.R_LQR;
% Equilibrium states
xEqlin = [xEq;zeros(cte.n/2,1)];
uEqlin = uEq;
end

%% xdotContinuous - Description
% This function computes the derivative of the state vector using the non 
% linear dynamics of the model, for a given state and control vector.
% Input:    - x: state vector 
%           - u: actuation vector
% Output:   - xdot: derivative of the state vector
function xdot = xdotContinuous(x,u,cte)
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
% Compute time constants of each tank
T = zeros(cte.n,1);
for i = 1:cte.n
   T(i) = (cte.A(i)/cte.a(i))*sqrt(2*x_eq(i)/cte.g);
end
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

%% References
% [1] L. Pedroso, and P. Batista (xxx), Discrete-time decentralized linear 
% quadratic control for linear time-varying systems, Int J Robust Nonlinear
% Control, xxx;xx:x?x. [Submitted to journal]