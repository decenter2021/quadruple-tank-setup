function [K,P] = LQROneStepLTV(system,E,T)
%% Description
% This function computes the one-step LQR regulator gain for a window T
% Input:    - system: (T+1)x4 cell whose rows contain matrices A,B,Q and R 
%           for the whole window, i.e.,
%               - system{i,1} = A(k+i-1), i = 1,...,T
%               - system{i,2} = B(k+i-1), i = 1,...,T
%               - system{i,3} = Q(k+i-1), i = 1,...,T+1
%               - system{i,4} = R(k+i-1), i = 1,...,T
%           Note that for the entry (T+1) only Q is used.
%           - E: sparsity pattern
%           - T: window length (T gains computed)
% Output:   - K: Tx1 cell of gains for the whole window,i.e.,
%                 K{i,1}, tau = k+i-1,...,k+i-2+T
%           - P: (T+1)x1 cell of P matrices for the whole window, i.e.,
%                 P{i,1}, tau = k+i-1,...,k+T
% Important notes: 
%           - output gain corresponds to the control law: u(k)=-K(k)*x(k)

%% Gain computation
n = size(system{1,1},1); % Get value of n from the size of A 
m = size(system{1,2},2); % Get value of n from the size of B 
K = cell(T,1);

% Init for code generation
% Variable p necessary for code generation
p = T+1;
P = cell(p,1); % Initialize cell arrays
for i = 1:p
     P{i,1} = zeros(n,n);
end
for i = 1:T
    K{i,1} = zeros(2,6); % 2,6 intead of m,n because of coder
end
P{T+1,1} = system{T+1,3}; % terminal condition  
for k = T:-1:1
   S = system{k,4}+system{k,2}'*P{k+1,1}*system{k,2};
   for j = 1:n
        % Compute L and M for code genaration
        L = zeros(n);
        L(j,j) = 1; % Generate matrix L_i
        M = zeros(m);
        for i = 1:m % Gererate matrix M_i
            if E(i,j) ~= 0
                M(i,i) = 1;
            end
        end
        % Compute the ith term of the summation  
        K{k,1} = K{k,1} + ...
         (eye(m)-M+M*S*M)\M*system{k,2}'*P{k+1,1}*system{k,1}*L;
   end
   % Update P
   P{k,1} = system{k,3}+K{k,1}'*system{k,4}*K{k,1}+...
       (system{k,1}-system{k,2}*K{k,1})'*P{k+1,1}*(system{k,1}-system{k,2}*K{k,1});
end       
end