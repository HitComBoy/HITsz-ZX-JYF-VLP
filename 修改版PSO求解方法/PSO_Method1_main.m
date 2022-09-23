%*********************************************************************************************%
% Use the Particle Swarm Optimization(PSO) to calculate the position result in our method one.%
% And consider the effect of choice of particle num or circle num on the position result.     %
%*********************************************************************************************%
clear
clc
tic
m = 1;  %LED的伦勃朗阶数
M = 1;  %PD的伦勃朗阶数
Ar = 1; %PD的有效接收面积
N_led = [0 0 -1];     %LED的法线向量
P_led = [1.5 1.5 3];  %LED的位置
K = 3:3:9; %旋转姿态数的选取

%********For the num of partices, find the relationship between SNR and position accuracy*******%
num = 128;       %粒子数量
num_cycle = 100; %粒子群循环次数
P_pd = [1.5 1.5 2];               %设置一个固定的样本点

test_time = 20000;                     %在特定粒子群数量下，定位误差测试次数（最后取平均）



SNR_dB = 20:5:60;                  %不同信噪比dB
SNR = 10.^(SNR_dB/10);              %不同信噪比dB
Err = zeros(size(K,2),size(SNR_dB, 2));       %在每个坐标下de定位误差
MidErr =zeros(1,test_time);         %纪录每次测试下的误差

for i=1:size(K,2)
    N_pd = Facing_Vector(P_pd, P_led, K(i));  %PD旋转过程中的法线向量
    for j=1:size(SNR,2)
            Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的功率
            Var = mean(Power)/SNR(j);   %在该测试点下，PD旋转过程中收到的噪声干扰
            parfor k=1:test_time
                Power_noise = Power +sqrt(Var)*randn(1,K(i));     %给接收功率加上噪声
                Estimated_Pos = PSO_Method1(Power_noise, P_led, N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置
                Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
                MidErr(k) = Err;                                                                     %累加，最后求平均
            end
            Err(i,j) = mean(MidErr); %在该测试点下，平均定位误差    
    end
end

figure(1)
semilogy(SNR_dB,Err(1,:))
hold on
semilogy(SNR_dB,Err(2,:))
hold on
semilogy(SNR_dB,Err(3,:))
hold off

grid on
xlabel('SNR')
ylabel('Position accuracy (m)')
title('The relation ship between SNR and position accuracy')
legend('K=3','K=6','K=9')

%********For the num of partices, find the relationship between K and position accuracy*******%
K = 3:3:30; %旋转姿态数的选取
SNR_dB = 40:10:60;                  %不同信噪比dB
SNR = 10.^(SNR_dB/10);              %不同信噪比dB
Err = zeros(size(SNR_dB, 2),size(K,2));       %在每个坐标下de定位误差(K=3)
MidErr =zeros(1,test_time);         %纪录每次测试下的误差

for i=1:size(SNR,2)
    for j=1:size(K,2)
            N_pd = Facing_Vector(P_pd, P_led, K(j));  %PD旋转过程中的法线向量
            Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的功率
            Var = mean(Power)/SNR(i);   %在该测试点下，PD旋转过程中收到的噪声干扰
            parfor k=1:test_time
                Power_noise = Power +sqrt(Var)*randn(1,K(j));     %给接收功率加上噪声
                Estimated_Pos = PSO_Method1(Power_noise, P_led, N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置
                Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
                MidErr(k) = Err;                                                                     %累加，最后求平均
            end
            Err(i,j) = mean(MidErr); %在该测试点下，平均定位误差    
    end
end

figure(2)
semilogy(K,Err(1,:))
hold on
semilogy(K,Err(2,:))
hold on
semilogy(K,Err(3,:))
hold off

grid on
xlabel('K')
ylabel('Position accuracy (m)')
title('The relation ship between K and position accuracy')
legend('SNR=40dB','SNR=50dB','SNR=60dB')
toc