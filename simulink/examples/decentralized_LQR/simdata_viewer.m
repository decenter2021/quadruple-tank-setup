function simdata_viewer(dataset)

datasetName = sprintf("./data/quadruple_example_decentralizedLQG_data_%s.mat",dataset);
data = load(datasetName);
data = data.quadruple_example_decentralizedLQG_data;
t = data.Time;
x = data.Data(1:4,1,:);
x = reshape(x,[4,size(t,1)])';
y = data.Data(5:8,1,:);
y = reshape(y,[4,size(t,1)])';
x_hat = data.Data(9:12,1,:);
x_hat = reshape(x_hat,[4,size(t,1)])';
u = data.Data(13:14,1,:);
u = reshape(u,[2,size(t,1)])';
ref = data.Data(15:16,1,:);
ref = reshape(ref,[2,size(t,1)])';
%simout.Data = permute(simout.Data,[1 3 2]);

%% One plot for all
figure;
hold on;
set(gca,'FontSize',35);
ylabel("$h(t)$ (cm)",'Interpreter','latex');
xlabel('$t$ (s)','Interpreter','latex');
for tank = 1:4
    p = plot(t,x(:,tank));
    p.LineWidth = 3;
    if tank <= 2
         p = plot(t,ref(:,tank),'--');
         p.LineWidth = 3;
    end
end
legend('h_1','ref_1','h_2','ref_2','h_3','h_4');
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
hold off;  

figure;
hold on;
set(gca,'FontSize',35);
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
ylabel("$u(t)$ (V)",'Interpreter','latex');
xlabel('$t$ (s)','Interpreter','latex');
for pump = 1:2 
    p = plot(t,u(:,pump));
    p.LineWidth = 3;
end
legend('u_1','u_2');
hold off;

%% One plot for each
% for tank = 1:4
%     figure;
%     hold on;
%     set(gca,'FontSize',35);
%     ylabel(sprintf("$h_%d$ (cm)",tank),'Interpreter','latex');
%     xlabel('$t$ (s)','Interpreter','latex');
%     p = plot(t,x(:,tank));
%     p.LineWidth = 3;
%     if tank <= 2
%          p = plot(t,ref(:,tank));
%          p.LineWidth = 3;
%          legend('One-step','Reference');
%     else
%         legend('One-step');
%     end
%     ax = gca;
%     ax.XGrid = 'on';
%     ax.YGrid = 'on';
%     hold off;   
% end
% 
% for pump = 1:2
%     figure;
%     hold on;
%     set(gca,'FontSize',35);
%     ax = gca;
%     ax.XGrid = 'on';
%     ax.YGrid = 'on';
%     ylabel(sprintf("$u_%d$ (V)",pump),'Interpreter','latex');
%     xlabel('$t$ (s)','Interpreter','latex');
%     p = plot(t,u(:,pump));
%     p.LineWidth = 3;
%     legend('One-step');
% end

