%*********************************************************************************************%
% Use the Particle Swarm Optimization(PSO) to calculate the position result in our method one.%
% And consider the effect of choice of particle num or circle num on the position result.     %
%*********************************************************************************************%
clear
clc
tic
m = 1; %LED的伦勃朗阶数
M = 1; %PD的伦勃朗阶数
Ar = 1; %PD的有效接收面积
N_led = [0 0 -1];     %LED的法线向量
P_led = [1.5 1.5 3];  %LED的位置

%*For static num and num_cycle, find the relation ship between position and position accuracy*%
Pos(1,:) = 0:0.5:3; %生成测试点的x坐标
Pos(2,:) = 0:0.5:3; %生成测试点的y坐标
Pos(3,:) = 0.5;     %生成测试点的z坐标(为了便于展示，暂时固定z坐标)
test_time = 10;     %在每个坐标下，定位误差测试次数（最后取平均）
Err_1 = zeros(size(Pos,2),size(Pos,2)); %在每个坐标下de定位误差(64 100)
Err_2 = zeros(size(Pos,2),size(Pos,2)); %在每个坐标下de定位误差(32 100)
Err_3 = zeros(size(Pos,2),size(Pos,2)); %在每个坐标下de定位误差(16 100)
MidErr =zeros(1,test_time);             %纪录每次测试下的误差

num = 64;        %粒子数量
num_cycle = 100; %粒子群循环次数
for i=1:size(Pos,2)
    for j=1:size(Pos,2)
        P_pd = [Pos(1,i) Pos(2,j) Pos(3,1)]; %生成测试点
        N_pd = Facing_Vector(P_pd, P_led);   %PD旋转过程中的法线向量
        parfor k=1:test_time
            Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);                         %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
            Estimated_Pos = PSO_Method1(Power, P_led, N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置
            Err = norm((P_pd - Estimated_Pos), 2);                                            %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                  %累加，最后求平均
        end
        Err_1(i,j) = mean(MidErr); %在该测试点下，平均定位误差
    end    
end

num = 4;       %粒子数量
num_cycle = 100; %粒子群循环次数
for i=1:size(Pos,2)
    for j=1:size(Pos,2)
        P_pd = [Pos(1,i) Pos(2,j) Pos(3,1)]; %生成测试点
        N_pd = Facing_Vector(P_pd, P_led);   %PD旋转过程中的法线向量
        parfor k=1:test_time
            Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);                         %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
            Estimated_Pos = PSO_Method1(Power, P_led, N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置
            Err = norm((P_pd - Estimated_Pos), 2);                                            %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                  %累加，最后求平均
        end
        Err_2(i,j) = mean(MidErr); %在该测试点下，平均定位误差
    end    
end

mesh(Pos(1,:),Pos(2,:),Err_1,'edgecolor','g')
hold on
mesh(Pos(1,:),Pos(2,:),Err_2,'edgecolor','m')
hold off
grid on
colorbar
xlabel('X position')
ylabel('Y position')
zlabel('MSE')
title('The relation ship between position and position accuracy')
legend('Particle:64 + Cycle:100','Particle:4 + Cycle:100')
toc