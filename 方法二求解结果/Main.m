%*********************************************************************************************%
% Use the Method 2 to calculate the position result.                                          %
% And consider the effect of SNR or orientation num on the position result.                   %
%*********************************************************************************************%
clear 
clc
tic
%*******************************Basical Parameter Setting*************************************%
m = 1; %LED的伦勃朗阶数
M = 1; %PD的伦勃朗阶数
Ar = 1; %PD的有效接收面积
K = 4;  %旋转过程中的姿态的选取
N_led = [0 0 -1];     %LED的法线向量
P_led = [1.5 1.5 3];  %LED的位置
P_pd = [1.5 1.5 0.5];                  %设置一个固定的样本点
N_pd = Facing_Vector(P_pd, P_led, K);  %PD旋转过程中的法线向量
SNR_dB = 30:10:100;                  %不同信噪比dB
SNR = 10.^(SNR_dB/10);              %不同信噪比
test_time = 20000;                    %定位误差测试次数（最后取平均）
Err = zeros(size(K,2),size(SNR,2)); %定位误差
%**********************************Monte Carlo Method****************************************%
for i=1:size(SNR,2)
        Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的功率
        Var = 0*mean(Power)/SNR(i);   %在该测试点下，PD旋转过程中收到的噪声干扰
        parfor k=1:test_time
            Power_noise = Power + sqrt(Var)*randn(1,K);     %给接收功率加上噪声
            Estimated_Pos = methodtwo(Power_noise, P_led, N_pd, 0.5); %在该测试点下，方法二所得到的估计位置
            Estimated_Pos = Estimated_Pos';
            Err = norm((P_pd - Estimated_Pos), 2).^2;                                               %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                     %累加，最后求平均
        end
        Err(i) = mean(MidErr); %在该测试点下，平均定位误差    
end

figure(1)
semilogy(SNR_dB,Err)

grid on
xlabel('SNR')
ylabel('Position accuracy')
title('The relation ship between SNR and position accuracy')
legend('Method2')
toc