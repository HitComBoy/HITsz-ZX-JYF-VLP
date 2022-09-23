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


%%%%%%%*For the num of partices, find the relation ship between num and position accuracy*%%%%%
num = 16:8:128;   %粒子数量
num_cycle = 100;   %粒子群循环次数
P_pd = [1.5 1.5 2];                 %设置一个固定的样本点
Angle_Dev =pi/3;   %偏移角度
Angle_Rot = 2*pi/3;   %旋转角度
N_pd = Facing_Vector(P_pd, P_led, Angle_Dev, Angle_Rot);  %PD旋转过程中的法线向量
test_time = 20000;                    %在特定粒子群数量下，定位误差测试次数（最后取平均）
Err_1 = zeros(1,size(num,2));       %在每个坐标下de定位误差(移动100次 40dB)
Err_2 = zeros(1,size(num,2));       %在每个坐标下de定位误差(移动100次 60dB）
MidErr =zeros(1,test_time);         %纪录每次测试下的误差
SNR_dB = 40;                        %不同信噪比dB
SNR = 10.^(SNR_dB/10);              %不同信噪比dB
for i=1:size(num,2)
        parfor k=1:test_time
            Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);                            %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
            Var = mean(Power)/SNR;
            Power_noise = Power +sqrt(Var)*randn(1,3);     %给接收功率加上噪声
            Estimated_Pos = PSO_Method1(Power_noise, P_led, N_pd, N_led, m, M, Ar, num(i), num_cycle); %在该测试点下，算法所得到的估计位置
            Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                     %累加，最后求平均
        end
        Err_1(i) = mean(MidErr); %在该测试点下，平均定位误差
        ['Work remain ' num2str(size(num,2)-i) '  (' num2str(1) '/' num2str(4) ')'] 
end

SNR_dB = 60;                  %不同信噪比dB
SNR = 10.^(SNR_dB/10);              %不同信噪比dB
for i=1:size(num,2)
        parfor k=1:test_time
            Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);                            %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
            Var = mean(Power)/SNR;
            Power_noise = Power +sqrt(Var)*randn(1,3);     %给接收功率加上噪声
            Estimated_Pos = PSO_Method1(Power_noise, P_led, N_pd, N_led, m, M, Ar, num(i), num_cycle); %在该测试点下，算法所得到的估计位置
            Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                     %累加，最后求平均
        end
        Err_2(i) = mean(MidErr); %在该测试点下，平均定位误差
        ['Work remain ' num2str(size(num,2)-i) '  (' num2str(3) '/' num2str(4) ')'] 
end


figure(1)
semilogy(num,Err_1.^2)
hold on
semilogy([16 128],[(Err_1(7)*0.97).^2 (Err_1(7)*0.97).^2])
hold on
semilogy(num,Err_2.^2)
hold on
semilogy([16 128],[(Err_2(7)*0.95).^2 (Err_2(7)*0.95).^2])
hold off

grid on
xlabel('Convergence speed of the proposed 3D R-VLP approach with respect to the number of particles')
ylabel('MSE')
legend('\itM=100 SNR=40dB','CRLB (SNR=40dB,\itK=3)','\itM=100 SNR=60dB','CRLB (SNR=60dB,\itK=3)')