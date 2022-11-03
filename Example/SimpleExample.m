% 卡尔曼滤波案例（一维度）：车辆定速巡航(10m/s)的X位置
% 1) 实际数据
%    X = X + T*V 
%    Y = X
X = 0; V0 = 10; T = 5;
SequenceT = linspace(0, T, 500);
SequenceX = SequenceT * V0;
figure(1); subplot(2, 2, 1);
plot(SequenceT, SequenceX, '-'); xlim([0, 5]); ylim([0, 50]); xlabel('T'); ylabel('X');
legend('Actual Data', 'Location', 'southeast');
% 2) 观测数据
muN    = [0.0];
SigmaN = [0.2];
for i = 1 : length(SequenceX)
    SequenceX_Obs(1,i) =  SequenceX(1,i) + sqrt(SigmaN) * randn(1,1) + muN;
end
figure(1); subplot(2, 2, 2);
plot(SequenceT, SequenceX_Obs, '+'); xlim([0, 5]); ylim([0, 50]); xlabel('T'); ylabel('X');
legend('Observed Data', 'Location', 'southeast');
% 3) 滤波后的数据
X0 = 0;  Sigma0 = 0.2; Q = 0.0; R = 0.2;
A  = 1;  B = 0.01; U = 10; H = 1;
for i = 1 : length(SequenceX_Obs)
    Mu1 = A * X0 + B * U;
    Sigma1 = A * Sigma0 * A' + Q;
    K = (Sigma1 * H') / (H * Sigma1 * H' + R);
    X0 = Mu1 + K * (SequenceX_Obs(i) - H * Mu1);
    Sigma0 = (1 - K * H) * Sigma1;
    SequenceX_klm(1,i) = X0;
end
figure(1); subplot(2, 2, 3);
plot(SequenceT, SequenceX_klm, '+'); xlim([0, 5]); ylim([0, 50]); xlabel('T'); ylabel('X');
legend('Filtered Data', 'Location', 'southeast');
% 4) 对比图
figure(1); subplot(2, 2, 4);
plot(SequenceT, SequenceX, 'k-'); hold on;
plot(SequenceT, SequenceX_Obs, 'r+'); hold on;
plot(SequenceT, SequenceX_klm, 'g*'); hold on;
xlim([0, 5]); ylim([0, 50]); xlabel('T'); ylabel('X');
legend('Actual Data', 'Observed Data', 'Filtered Data', 'Location', 'southeast');