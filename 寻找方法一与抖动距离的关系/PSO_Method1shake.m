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
Angle_Dev =pi/3;   %偏移角度
Angle_Rot = 2*pi/3;   %旋转角度
%********For the num of partices, find the relationship between SNR and position accuracy*******%
num = 128;       %粒子数量
num_cycle = 100; %粒子群循环次数
P_pd = [1.5 1.5 2];               %设置一个固定的样本点

test_time = 200;                     %在特定粒子群数量下，定位误差测试次数（最后取平均）

MidErr =zeros(1,test_time);         %纪录每次测试下的误差
SNR_dB = [40 60];                  %不同信噪比dB
SNR = 10.^(SNR_dB/10);              %不同信噪比dB
Shakedistance = 0:0.01:0.1;       %不同抖动距离


Err = zeros(size(SNR,2),size(Shakedistance,2));       %在每个坐标下de定位误差(128+100)

for i=1:size(SNR,2)
        N_pd = Facing_Vector( P_pd, P_led, Angle_Dev, Angle_Rot);  %PD旋转过程中的法线向量
        Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的功率(无抖动理想状态下)
        Var = mean(Power)/SNR(i) ;   %在该测试点下，PD旋转过程中收到的噪声干扰
        for j=1:size(Shakedistance,2)
            parfor k=1:test_time
                R_Power1 = Theory_Power(P_pd, P_led, N_pd(1,:), N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的第一个功率
                Shakevec = randn(1,3);
                Shakevec = Shakevec/norm(Shakevec,2);
                Shakevec = Shakedistance(j)*Shakevec; %获得随机位移向量
                R_Power2 = Theory_Power(P_pd+Shakevec, P_led, N_pd(2,:), N_led, m, M, Ar);    %在该测试点下，经过随机抖动后，PD旋转过程中接收到的第二个功率
                Shakevec = randn(1,3);
                Shakevec = Shakevec/norm(Shakevec,2);
                Shakevec = Shakedistance(j)*Shakevec; %获得随机位移向量
                R_Power3 = Theory_Power(P_pd+Shakevec, P_led, N_pd(3,:), N_led, m, M, Ar);    %在该测试点下，经过随机抖动后，PD旋转过程中接收到的第三个功率
                R_Power = [R_Power1 R_Power2 R_Power3];%在抖动情况下获得的实际功率
                Power_noise = R_Power +sqrt(Var)*randn(1,3);     %给接收功率加上噪声
                Estimated_Pos = PSO_Method1(Power_noise, P_led, N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置
                Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
                MidErr(k) = Err;                                                                     %累加，最后求平均
            end
            Err(i,j) = mean(MidErr); %在该测试点下，平均定位误差
            ['Work remain' num2str(size(SNR,2)*size(Shakedistance,2)-((i-1)*size(Shakedistance,2)+j))] 
        end

end

figure(1)
semilogy(Shakedistance,Err(1,:).^2,'Color','#0072BD','LineStyle','-')
hold on
semilogy(Shakedistance,repmat(Err(1,1).^2,1,size(Shakedistance,2)),'Color','#0072BD','LineStyle','--')
hold on
semilogy(Shakedistance,Err(2,:).^2,'Color','#D95319','LineStyle','-')
hold on
semilogy(Shakedistance,repmat(Err(2,1).^2,1,size(Shakedistance,2)),'Color','#D95319','LineStyle','--')
hold on
semilogy(Shakedistance,Err(3,:).^2,'Color','#77AC30','LineStyle','-')
hold on
semilogy(Shakedistance,repmat(Err(3,1).^2,1,size(Shakedistance,2)),'Color','#77AC30','LineStyle','--')
hold off
grid on
title('The relation ship between Shanke distance and position accuracy')
legend('SNR 40dB','SNR 40dB(without shake)','SNR 60dB','SNR 60dB(without shake)','SNR 80dB''SNR 80dB(without shake)')
xlabel('Shake distance')
ylabel('Position accuracy')
toc