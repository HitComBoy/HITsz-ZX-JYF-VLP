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


%********For the num of partices, find the relationship between SNR and position accuracy*******%
num = 40;       %粒子数量
num_cycle = 40; %粒子群循环次数
P_pd = [1.5 1.5 1.5];               %设置一个固定的样本点
N_pd = Facing_Vector(P_pd, P_led);  %PD旋转过程中的法线向量
test_time = 2000;                     %在特定粒子群数量下，定位误差测试次数（最后取平均）
Err_1 = zeros(1,size(num,2));       %在每个坐标下de定位误差(128+100)
Err_2 = zeros(1,size(num,2));       %在每个坐标下de定位误差(128+50）
Err_3 = zeros(1,size(num,2));       %在每个坐标下de定位误差(128+10）
MidErr =zeros(1,test_time);         %纪录每次测试下的误差
SNR_dB = 0:10:50;                  %不同信噪比dB
SNR = 10.^(SNR_dB/10);              %不同信噪比dB
for i=1:size(SNR,2)
        Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的功率
        Var = Power/SNR(i);   %在该测试点下，PD旋转过程中收到的噪声干扰
        for k=1:test_time
            Power_noise = Power +sqrt(Var).*randn(1,3);     %给接收功率加上噪声
            Estimated_Pos = PSO_MethodE(Power_noise, P_led, N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置
            Err = norm((P_pd - Estimated_Pos), 2)^2;                                               %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                     %累加，最后求平均
        end
        Err_1(i) = mean(MidErr); %在该测试点下，平均定位误差    
        ['Position(1/2),remain' num2str(size(SNR,2)-i)]  
end

for i=1:size(SNR,2)
        Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的功率
        Var = Power/SNR(i);   %在该测试点下，PD旋转过程中收到的噪声干扰
        parfor k=1:test_time
            Power_noise = Power +sqrt(Var).*randn(1,3);     %给接收功率加上噪声
            Estimated_Pos = PSO_Method(Power_noise, P_led, N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置
            Err = norm((P_pd - Estimated_Pos), 2).^2;                                               %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                     %累加，最后求平均
        end
        Err_2(i) = mean(MidErr); %在该测试点下，平均定位误差
        ['Position(2/2),remain' num2str(size(SNR,2)-i)]          
end


figure(1)
semilogy(SNR_dB,Err_1)
hold on
semilogy(SNR_dB,Err_2)
hold off

grid on
xlabel('SNR')
ylabel('MSE')
title('The relation ship between SNR and position accuracy')
legend('Enhanced-PSO','PSO')