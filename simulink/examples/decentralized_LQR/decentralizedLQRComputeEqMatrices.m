function decentralizedLQRComputeEqMatrices()
%% Load quandruple-tank process paramenters
cte = quadrupleLoadParameters();
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
save('EqMatrices.mat','alpha','beta');
end

