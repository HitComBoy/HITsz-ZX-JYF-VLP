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


%%%%%%%%%*For the num of Cycle, find the relation ship between num and position accuracy*%%%%%%
num_cycle = 10:1:20; %粒子群循环次数
num = 64;   %粒子数量
P_pd = [1.5 1.5 2];               %设置一个固定的样本点
Angle_Dev =pi/3;   %偏移角度
Angle_Rot = 2*pi/3;   %旋转角度
N_pd = Facing_Vector( P_pd, P_led(1,:), Angle_Dev, Angle_Rot);  %PD旋转过程中的法线向量
test_time = 20000;                   %在特定粒子群数量下，定位误差测试次数（最后取平均）
Err_1 = zeros(1,size(num_cycle,2));       %在每个坐标下de定位误差(粒子数128)
Err_2 = zeros(1,size(num_cycle,2));       %在每个坐标下de定位误差(粒子数64）
MidErr =zeros(1,test_time);         %纪录每次测试下的误差

SNR_dB = 40;                  %不同信噪比dB
SNR = 10.^(SNR_dB/10);              %不同信噪比dB
for i=1:size(num_cycle,2)
        Power = Theory_Power(P_pd, P_led(1,:), N_pd, N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的功率(无抖动理想状态下)
        Var = Power/SNR;   %在该测试点下，PD旋转过程中收到的噪声干扰
        parfor k=1:test_time
            Power_noise = Power +sqrt(Var).*randn(1,3);     %给接收功率加上噪声
            Estimated_Pos = PSO_Method1(Power_noise, P_led, N_pd, N_led, m, M, Ar, num, num_cycle(i)); %在该测试点下，算法所得到的估计位置
            Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                     %累加，最后求平均
        end
        Err_1(i) = mean(MidErr); %在该测试点下，平均定位误差    
end

SNR_dB = 60;                  %不同信噪比dB
SNR = 10.^(SNR_dB/10);              %不同信噪比dB
for i=1:size(num_cycle,2)
        Power = Theory_Power(P_pd, P_led(1,:), N_pd, N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的功率(无抖动理想状态下)
        Var = Power/SNR;   %在该测试点下，PD旋转过程中收到的噪声干扰
        parfor k=1:test_time
            Power_noise = Power +sqrt(Var).*randn(1,3);     %给接收功率加上噪声
            Estimated_Pos = PSO_Method1(Power_noise, P_led, N_pd, N_led, m, M, Ar, num, num_cycle(i)); %在该测试点下，算法所得到的估计位置
            Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                     %累加，最后求平均
        end
        Err_2(i) = mean(MidErr); %在该测试点下，平均定位误差    
end

figure(1)
semilogy(num_cycle,Err_1.^2)
hold on
semilogy([10 20],[Err_1(11).^2*0.95 Err_1(11).^2*0.95])
hold on
semilogy(num_cycle,Err_2.^2)
hold on
semilogy([10 20],[Err_2(11).^2*0.95 Err_2(11).^2*0.95])
hold off
grid on
xlabel('The times of Cycle')
ylabel('MSE')
legend('N = 128 SNR40dB','CRLB SNR40dB','N = 128 SNR60dB','CRLB SNR60dB')
toc