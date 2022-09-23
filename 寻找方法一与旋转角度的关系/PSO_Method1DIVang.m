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
Angle_Dev = pi/6:pi/60:pi/3;   %偏移角度
Angle_Rot = pi/6:pi/18:5*pi/18;   %旋转角度
%********For the num of partices, find the relationship between SNR and position accuracy*******%
num = 128;       %粒子数量
num_cycle = 100; %粒子群循环次数
P_pd = [1.5 1.5 0.5];               %设置一个固定的样本点

test_time = 20;                     %在特定粒子群数量下，定位误差测试次数（最后取平均）
Err = zeros(size(Angle_Dev,2),size(Angle_Rot,2));       %在每个坐标下de定位误差(128+100)
MidErr =zeros(1,test_time);         %纪录每次测试下的误差
SNR_dB = 60;                  %不同信噪比dB
SNR = 10.^(SNR_dB/10);              %不同信噪比dB
for i=1:size(Angle_Dev,2)
        for j=1:size(Angle_Rot,2)
            N_pd = Facing_Vector( P_pd, P_led, Angle_Dev(i), Angle_Rot(j));  %PD旋转过程中的法线向量
            Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的功率
            Var = mean(Power)/SNR ;   %在该测试点下，PD旋转过程中收到的噪声干扰
            parfor k=1:test_time
                Power_noise = Power +sqrt(Var)*randn(1,3);     %给接收功率加上噪声
                Estimated_Pos = PSO_Method1(Power_noise, P_led, N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置
                Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
                MidErr(k) = Err;                                                                     %累加，最后求平均
            end
            Err(i,j) = mean(MidErr); %在该测试点下，平均定位误差
        end
        ['Work remain' num2str(size(Angle_Dev,2)*size(Angle_Rot,2)-((i-1)*size(Angle_Rot,2)+j))] 
end

figure(1)
semilogy(Angle_Dev*4-pi/2,Err(:,1).^2)
hold on
semilogy(Angle_Dev*4-pi/2,Err(:,2).^2)
hold on
semilogy(Angle_Dev*4-pi/2,Err(:,3).^2)
hold off
grid on
title('The relation ship between Deviation Angle and position accuracy')
legend('Rotation angle 30','Rotation angle 60','Rotation angle 90')
toc