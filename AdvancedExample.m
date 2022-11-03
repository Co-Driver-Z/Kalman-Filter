%% 1、初始化相关变量
clc; clear; close all;
X = 0; Y = 0; Theta = 0; V = 10;                                           % 车辆的初始状态
Plant.v = V; Plant.l = 2.92; Plant.dt = 0.05; H = 110/(0.05*V);            % 设置仿真工况
D0 = 2; D1 = 4; D2 = 6; D3 = 8; D4 = 10; D5 = 12;                          % 设置关键索引
M=zeros(12, 1); M([1, 2])=[Y; Theta]; M([3, 4])=[Y; Theta]; M([5, 6])=0;   % 设置增广矩阵用于求解控制量(初始时目标点为 [0; 0])
policy.maxU = [0.5236];                                                    % 
policy.p.w = [-0.9501, -2.7402, -0.0123, -0.1153,  0.0250,  0.1631];        
Poli = [7, 8, 9, 10, 11, 12];                                              % 控制器所需要的变量
State1=zeros(5, H+1); State1(1,1)=X; State1(2,1)=Y; State1(3,1)=Theta; State1(4,1)=V;

%% 2.1、获得真实数据
for i = 1 : 1 : H
    % 0) Clean up m variables
    M(D2+1:D5) = 0;
    % 1) Target state
    [Xdes, Ydes, Thetades, Kdes] = BinarySearchTarget(X); Md = zeros(2, 1); Md(1) = Ydes; Md(2) = Thetades;
    % 2) Augment the error at the current moment
    A = eye(D0); ii = 1:D0; kk = D2+1:D3; M(kk) = A * M(ii) - Md;
    % 3) Augment the integral of the error at the current moment
    A = [eye(D0), eye(D0)*Plant.dt]; ii = D1+1:D3; kk = D3+1:D4; M(kk) = A * M(ii);
    % 4) Augment the differential of the error at the current moment
    A = (1/Plant.dt)*[-eye(D0), eye(D0)]; ii = [D0+1:D1, D2+1:D3]; kk = D4+1:D5; M(kk) = A * M(ii);
    % 5) Compute distribution of the control signal
    U = policy.p.w * M(Poli); U = policy.maxU * (9 * sin(U) + sin(3 * U)) / 8;
    % 6) Calculate the state at the next moment according to dynamics
    [X, Y, Theta] = KineModel(X, Y, Theta, Plant, U);
    % 7) Update M Vector
    M(1) = Y; M(2) = Theta; M(3) = M(7); M(4) = M(8); M(5) = M(9); M(6) = M(10);  
    % 8) Storage state quantity
    State1(1,i+1)=X; State1(2,i+1)=Y; State1(3,i+1)=Theta; State1(4,i+1)=V; State1(5,i)=U;
end

%% 2.2、添加噪声，生成观测数据
State2 = zeros(5, H+1); muNxy = [0.0]; SigmaNxy = [0.01]; muNt = [0.0]; SigmaNt = [0.0002];
for i = 1 : 1 : H
    State2(1,i) = State1(1,i) + sqrt(SigmaNxy) * randn(1,1) + muNxy;
    State2(2,i) = State1(2,i) + sqrt(SigmaNxy) * randn(1,1) + muNxy;
    State2(3,i) = State1(3,i) + sqrt(SigmaNt)  * randn(1,1) + muNt;
    State2(4,i) = State1(4,i);
    State2(5,i) = State1(5,i);
end

%% 2.3、卡尔曼滤波
State3 = zeros(5, H+1);      State3(1,1) =  State2(1,1); State3(2,1) =  State2(2,1);
State3(3,1) =  State2(3,1);  State3(4,1) =  State2(4,1); State3(5,1) =  State2(5,1);
X0 = [State2(1,1); State2(2,1); State2(3,1)]; Sigma0 = diag([0.01; 0.01; 0.0002]); Q = diag([0.01; 0.01; 0.0002].^2); R = diag([0.01; 0.01; 0.0002]);
for i = 2 : 1 : H
    A = eye(length(X0)); B = Plant.dt*[cos(X0(3)); sin(X0(3)); tan(State2(5,i-1))/Plant.l]; U = State2(4,i-1); Hk = eye(length(X0));
    Mu1    = A * X0 + B * U;
    Sigma1 = A * Sigma0 * A' + Q;
    Mu2    = Hk * Mu1;
    Sigma2 = Hk * Sigma1 * Hk';
    Mu3    = eye(length(X0))/( eye(length(X0))/Sigma2 + eye(length(X0))/R) * (Sigma2\Mu2 + R\[State2(1,i); State2(2,i); State2(3,i)]);
    Sigma3 = eye(length(X0))/( eye(length(X0))/Sigma2 + eye(length(X0))/R);
    Mu4    = Hk \ Mu3;
    Sigma4 = (Hk \ Sigma3) * (eye(length(X0))/Hk)';
    X0 = Mu4;
    Sigma0 = Sigma4;
    State3(1,i) = Mu4(1);
    State3(2,i) = Mu4(2);
    State3(3,i) = Mu4(3);
    State3(4,i) = State2(4,i);
    State3(5,i) = State2(5,i);
end
%% 3、绘图
figure(100); load('DoubleLaneChangingTraj.mat');
subplot(2, 2, 1);
plot(Traj_X, Traj_Y, 'k--', 'linewidth', 3), hold on;
plot(State1(1, 1:H), State1(2, 1:H), 'color', [47, 137, 252]/255, 'linewidth', 2), hold on;
plot(State2(1, 1:H), State2(2, 1:H), 'color', [255, 222,  0]/255, 'linewidth', 2), hold on;
plot(State3(1, 1:H), State3(2, 1:H), 'color', [130, 205, 71]/255, 'linewidth', 2), hold on;
title('X-Y'); xlabel('X(m)','FontSize',12,'FontName','Times'); ylabel('Y(m)','FontSize',12,'FontName','Times'); xlim([0, 110]); ylim([-0.5, 5]);
legend('Ref', 'Actual Data', 'Observed Data', 'Filtered Data',  'Location', 'northeast');

subplot(2, 2, 2);
plot(Traj_X, Traj_Y, 'k--', 'linewidth', 3), hold on;
plot(State1(1, 1:H), State1(2, 1:H), 'color', [47, 137, 252]/255, 'linewidth', 2), hold on;
plot(State2(1, 1:H), State2(2, 1:H), 'color', [255, 222,  0]/255, 'linewidth', 2), hold on;
plot(State3(1, 1:H), State3(2, 1:H), 'color', [130, 205, 71]/255, 'linewidth', 2), hold on;
title('X-Y'); xlabel('X(m)','FontSize',12,'FontName','Times'); ylabel('Y(m)','FontSize',12,'FontName','Times'); xlim([33, 50]); ylim([2.5, 4.3]);
legend('Ref', 'Actual Data', 'Observed Data', 'Filtered Data',  'Location', 'northeast');

subplot(2, 2, 3);
plot(1:H, zeros(1,H), 'k--', 'linewidth', 3), hold on;
plot([1:H]*Plant.dt, State1(2, 1:H), 'color', [47, 137, 252]/255, 'linewidth', 2), hold on;
plot([1:H]*Plant.dt, State2(2, 1:H), 'color', [255, 222,  0]/255, 'linewidth', 2), hold on;
plot([1:H]*Plant.dt, State3(2, 1:H), 'color', [130, 205, 71]/255, 'linewidth', 2), hold on;
title('T-Y'); xlabel('T(s)','FontSize',12,'FontName','Times'); ylabel('Y(m)','FontSize',12,'FontName','Times'); xlim([0, H*Plant.dt]); ylim([-0.5, 5]);
legend('Ref', 'Actual Data', 'Observed Data', 'Filtered Data', 'Location', 'northeast');

subplot(2, 2, 4);
plot(1:H, zeros(1,H), 'k--', 'linewidth', 3), hold on;
plot([1:H]*Plant.dt, State1(3, 1:H), 'color', [47, 137, 252]/255, 'linewidth', 2), hold on;
plot([1:H]*Plant.dt, State2(3, 1:H), 'color', [255, 222,  0]/255, 'linewidth', 2), hold on;
plot([1:H]*Plant.dt, State3(3, 1:H), 'color', [130, 205, 71]/255, 'linewidth', 2), hold on;
title('T-Theta'); xlabel('T(s)','FontSize',12,'FontName','Times'); ylabel('Theta(m)','FontSize',12,'FontName','Times'); xlim([0, H*Plant.dt]); ylim([-0.4, 0.4]);
legend('Ref', 'Actual Data', 'Observed Data', 'Filtered Data', 'Location', 'northeast');



