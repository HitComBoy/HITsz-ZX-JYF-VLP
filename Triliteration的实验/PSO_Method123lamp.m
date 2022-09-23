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
P_pd = [1.5 1.5 0.5];               %设置一个固定的样本点
N_pd = [0 0 1];%传统三灯方法
%********For the num of partices, find the relationship between SNR and position accuracy*******%
num = 128;       %粒子数量
num_cycle = 100; %粒子群循环次数

test_time = 50;                     %在特定粒子群数量下，定位误差测试次数（最后取平均）

MidErr =zeros(1,test_time);         %纪录每次测试下的误差
SNR_dB = 40:20:80;                  %不同信噪比dB
SNR = 10.^(SNR_dB/10);              %不同信噪比dB

Err = zeros(2,size(SNR,2));       %在每个坐标下de定位误差(128+100)

P_led = [1.5 0.5 3; 2.36 2 3; 0.64 2 3];  %LED的位置相距离中心1m
for i=1:size(SNR,2)
        Power = [Theory_Power(P_pd, P_led(1,:), N_pd, N_led, m, M, Ar) Theory_Power(P_pd, P_led(2,:), N_pd, N_led, m, M, Ar) Theory_Power(P_pd, P_led(3,:), N_pd, N_led, m, M, Ar)];    %在该测试点下，PD旋转过程中接收到的双灯功率
        Var = Power/SNR(i) ;   %在该测试点下，PD旋转过程中收到的噪声干扰
            for k=1:test_time
                Power_noise = Power +sqrt(Var).*randn(1,3);     %给接收功率加上噪声
                Estimated_Pos = PSO_Method3lamptriliteration(Power_noise, [P_led(1,:);P_led(2,:);P_led(3,:)], N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置                                          
                MidErr(k) =  norm((P_pd - Estimated_Pos), 2);             %累加，最后求平均 
            end
            Err(1,i) = mean(MidErr); %在该测试点下，平均定位误差
            ['Work remain' num2str(size(SNR,2)-i) '  (' num2str(1) '/' num2str(2) ')'] 
end
P_led = [1.5 1 3; 1.933 1.75 3; 1.067 1.75 3];  %LED的位置相距中心0.5m

for i=1:size(SNR,2)
        Power = [Theory_Power(P_pd, P_led(1,:), N_pd, N_led, m, M, Ar) Theory_Power(P_pd, P_led(2,:), N_pd, N_led, m, M, Ar) Theory_Power(P_pd, P_led(3,:), N_pd, N_led, m, M, Ar)];    %在该测试点下，PD旋转过程中接收到的双灯功率
        Var = Power/SNR(i) ;   %在该测试点下，PD旋转过程中收到的噪声干扰
            for k=1:test_time
                Power_noise = Power +sqrt(Var).*randn(1,3);     %给接收功率加上噪声
                Estimated_Pos = PSO_Method3lamptriliteration(Power_noise, [P_led(1,:);P_led(2,:);P_led(3,:)], N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置         
                MidErr(k) = norm((P_pd - Estimated_Pos), 2);        %累加，最后求平均 
            end
            Err(2,i) = mean(MidErr); %在该测试点下，平均定位误差
            ['Work remain' num2str(size(SNR,2)-i) '  (' num2str(2) '/' num2str(2) ')'] 
end
Err
figure(1)
semilogy(SNR_dB,Err(1,:),'Color','#0072BD','LineStyle','-')
hold on
semilogy(SNR_dB,Err(2,:),'Color','#D95319','LineStyle','-')
hold off
grid on
% title('The relation ship between Shanke distance and position accuracy')
% legend('RSS(Lamp Distance = 1)','RSS(Lamp Distance = 0.5)')%,'SNR 60dB','SNR 60dB(without shake)','SNR 80dB''SNR 80dB(without shake)')
% xlabel('Shake distance')
% ylabel('Position accuracy')
toc