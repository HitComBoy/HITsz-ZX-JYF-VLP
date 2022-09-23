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

P_pd = [1.5 1.5 2];               %设置一个固定的样本点
Angle_Dev =pi/3;   %偏移角度
Angle_Rot = 2*pi/3;   %旋转角度
N_pd = Facing_Vector(P_pd, P_led, Angle_Dev, Angle_Rot);  %PD旋转过程中的法线向量
test_time = 20000;                  %在特定粒子群数量下，定位误差测试次数（最后取平均）
SNR_dB = 20:5:60;                   %不同信噪比dB
SNR = 10.^(SNR_dB/10);              %不同信噪比dB
Err_1 = zeros(1,size(SNR,2));       %在每个坐标下de定位误差(128+100)
Err_2 = zeros(1,size(SNR,2));       %在每个坐标下de定位误差(128+50）
Err_3 = zeros(1,size(SNR,2));       %在每个坐标下de定位误差(64+100）
MidErr =zeros(1,test_time);         %纪录每次测试下的误差
num =128;       %粒子数量
num_cycle =100; %粒子群循环次数
for i=1:size(SNR,2)
        Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的功率
        Var = Power/SNR(i);   %在该测试点下，PD旋转过程中收到的噪声干扰
        parfor k=1:test_time
            Power_noise = Power +sqrt(Var).*randn(1,3);     %给接收功率加上噪声
            Estimated_Pos = PSO_Method1(Power_noise, P_led, N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置
            Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                     %累加，最后求平均
        end
        Err_1(i) = mean(MidErr); %在该测试点下，平均定位误差
        ['Work remain ' num2str(size(SNR,2)-i) '  (' num2str(1) '/' num2str(3) ')'] 
end

num = 64;       %粒子数量
num_cycle = 100; %粒子群循环次数
for i=1:size(SNR,2)
        Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的功率
        Var = Power/SNR(i);   %在该测试点下，PD旋转过程中收到的噪声干扰
        parfor k=1:test_time
            Power_noise = Power +sqrt(Var).*randn(1,3);     %给接收功率加上噪声
            Estimated_Pos = PSO_Method1(Power_noise, P_led, N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置
            Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                     %累加，最后求平均
        end
        Err_2(i) = mean(MidErr); %在该测试点下，平均定位误差    
        ['Work remain ' num2str(size(SNR,2)-i) '  (' num2str(2) '/' num2str(3) ')'] 
end

num = 128;       %粒子数量
num_cycle = 50; %粒子群循环次数
for i=1:size(SNR,2)
        Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的功率
        Var = Power/SNR(i);   %在该测试点下，PD旋转过程中收到的噪声干扰
        parfor k=1:test_time
            Power_noise = Power +sqrt(Var).*randn(1,3);     %给接收功率加上噪声
            Estimated_Pos = PSO_Method1(Power_noise, P_led, N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置
            Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                     %累加，最后求平均
        end
        Err_3(i) = mean(MidErr); %在该测试点下，平均定位误差    
        ['Work remain ' num2str(size(SNR,2)-i) '  (' num2str(3) '/' num2str(3) ')'] 
end

Err_4 = zeros(1,size(SNR,2));       %在每个坐标下de定位误差(64+100）
num =128;       %粒子数量
num_cycle =100; %粒子群循环次数
for i=1:size(SNR,2)
        parfor k=1:test_time
            N_pd_temp = Facing_Vector(P_pd, P_led, Angle_Dev, (rand(1))*Angle_Rot);  %PD旋转过程中的法线向量
            Power = Theory_Power(P_pd, P_led, N_pd_temp, N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的功率
            Var = Power/SNR(i);   %在该测试点下，PD旋转过程中收到的噪声干扰
            Power_noise = Power +sqrt(Var).*randn(1,3);     %给接收功率加上噪声
            Estimated_Pos = PSO_Method1(Power_noise, P_led, N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置
            Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                     %累加，最后求平均
        end
        Err_4(i) = mean(MidErr); %在该测试点下，平均定位误差
        ['Work remain ' num2str(size(SNR,2)-i) '  (' num2str(1) '/' num2str(3) ')'] 
end


pi =  3.141592653589793;
CRLB = zeros(length(SNR),1);
A = (m+1)*Ar/(2*pi);
for i=1:length(SNR)
        Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);    %在该测试点下，PD旋转过程中接收到的功率
        Var = Power/SNR(i);
        delta = sqrt(Var);
    
        x =  P_pd(1) - P_led(1);
        y =  P_pd(2) - P_led(2);
        z =  P_pd(3) - P_led(3);

        n(:,:,1) = -transpose(N_led)*N_pd(1,:);
        n(:,:,2) = -transpose(N_led)*N_pd(2,:);
        n(:,:,3) = -transpose(N_led)*N_pd(3,:);
 
        H1 = Theory_Power(P_pd, P_led, N_pd(1,:), N_led, m, M, Ar);
        H2 = Theory_Power(P_pd, P_led, N_pd(2,:), N_led, m, M, Ar);
        H3 = Theory_Power(P_pd, P_led, N_pd(3,:), N_led, m, M, Ar);

        p1 = x*(A*n(1,1,3)*x+A*n(2,1,3)*y+A*n(3,1,3)*z)+y*(A*n(1,2,3)*x+A*n(2,2,3)*y+A*n(3,2,3)*z)+z*(A*n(1,3,3)*x+A*n(2,3,3)*y+A*n(3,3,3)*z);%same in nine element
        p2 = x*(A*n(1,1,2)*x+A*n(2,1,2)*y+A*n(3,1,2)*z)+y*(A*n(1,2,2)*x+A*n(2,2,2)*y+A*n(3,2,2)*z)+z*(A*n(1,3,2)*x+A*n(2,3,2)*y+A*n(3,3,2)*z);%same in nine element
        p3 = x*(A*n(1,1,1)*x+A*n(2,1,1)*y+A*n(3,1,1)*z)+y*(A*n(1,2,1)*x+A*n(2,2,1)*y+A*n(3,2,1)*z)+z*(A*n(1,3,1)*x+A*n(2,3,1)*y+A*n(3,3,1)*z);%same in nine element
        p4 = 12*x^2+4*y^2+4*z^2;
        p5 = A*n(1,1,3)*x+A*n(2,1,3)*y+A*n(3,1,3)*z+A*n(1,1,3)*x+A*n(1,2,3)*y+A*n(1,3,3)*z;
        p6 = A*n(1,1,2)*x+A*n(2,1,2)*y+A*n(3,1,2)*z+A*n(1,1,2)*x+A*n(1,2,2)*y+A*n(1,3,2)*z;
        p7 = A*n(1,1,1)*x+A*n(2,1,1)*y+A*n(3,1,1)*z+A*n(1,1,1)*x+A*n(1,2,1)*y+A*n(1,3,1)*z;
        p10 = x^2+y^2+z^2;
        p8 = p10*x^2+p10*y^2+p10*z^2;
        p9 = 2*x^3+2*x*y^2+2*x*z^2+2*x*p10;
        dxdx =(-(p7/p8-p3*p9/p8^2)^2-(H1-p3/p8)*(2*p9*p7/p8^2+p3*p4/p8^2-2*A*n(1,1,1)/p8-2*p3*p9^2/p8^3))/delta(1)^2 +...
              (-(p6/p8-p2*p9/p8^2)^2-(H2-p2/p8)*(2*p9*p6/p8^2+p2*p4/p8^2-2*A*n(1,1,2)/p8-2*p2*p9^2/p8^3))/delta(2)^2 +...
              (-(p5/p8-p1*p9/p8^2)^2-(H3-p1/p8)*(2*p9*p5/p8^2+p1*p4/p8^2-2*A*n(1,1,3)/p8-2*p1*p9^2/p8^3))/delta(3)^2;
        

        p1 = x*(A*n(1,1,3)*x+A*n(2,1,3)*y+A*n(3,1,3)*z)+y*(A*n(1,2,3)*x+A*n(2,2,3)*y+A*n(3,2,3)*z)+z*(A*n(1,3,3)*x+A*n(2,3,3)*y+A*n(3,3,3)*z);
        p2 = x*(A*n(1,1,2)*x+A*n(2,1,2)*y+A*n(3,1,2)*z)+y*(A*n(1,2,2)*x+A*n(2,2,2)*y+A*n(3,2,2)*z)+z*(A*n(1,3,2)*x+A*n(2,3,2)*y+A*n(3,3,2)*z);
        p3 = x*(A*n(1,1,1)*x+A*n(2,1,1)*y+A*n(3,1,1)*z)+y*(A*n(1,2,1)*x+A*n(2,2,1)*y+A*n(3,2,1)*z)+z*(A*n(1,3,1)*x+A*n(2,3,1)*y+A*n(3,3,1)*z);
        p4 = A*n(1,2,3)*x+A*n(2,2,3)*y+A*n(3,2,3)*z+A*n(2,1,3)*x+A*n(2,2,3)*y+A*n(2,3,3)*z;
        p5 = A*n(1,2,2)*x+A*n(2,2,2)*y+A*n(3,2,2)*z+A*n(2,1,2)*x+A*n(2,2,2)*y+A*n(2,3,2)*z;
        p6 = A*n(1,2,1)*x+A*n(2,2,1)*y+A*n(3,2,1)*z+A*n(2,1,1)*x+A*n(2,2,1)*y+A*n(2,3,1)*z;
        p7 = A*n(1,1,3)*x+A*n(2,1,3)*y+A*n(3,1,3)*z+A*n(1,1,3)*x+A*n(1,2,3)*y+A*n(1,3,3)*z;
        p8 = A*n(1,1,2)*x+A*n(2,1,2)*y+A*n(3,1,2)*z+A*n(1,1,2)*x+A*n(1,2,2)*y+A*n(1,3,2)*z;
        p9 = A*n(1,1,1)*x+A*n(2,1,1)*y+A*n(3,1,1)*z+A*n(1,1,1)*x+A*n(1,2,1)*y+A*n(1,3,1)*z;
        p10 = 8*x*y;
        p14 = x^2+y^2+z^2;
        p11 = p14*x^2+p14*y^2+p14*z^2;
        p12 = 2*y*x^2+2*y^3+2*y*z^2+2*y*p14;
        p13 = 2*x^3+2*x*y^2+2*x*z^2+2*x*p14;
        dxdy =(-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(H1-p3/p11)*(p13*p6/p11^2-(A*n(1,2,1)+A*n(2,1,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3))/delta(1)^2 + ...
              (-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(H2-p2/p11)*(p13*p5/p11^2-(A*n(1,2,2)+A*n(2,1,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3))/delta(2)^2 +...
              (-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-(H3-p1/p11)*(p13*p4/p11^2-(A*n(1,2,3)+A*n(2,1,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta(3)^2 ;

        p1 = x*(A*n(1,1,3)*x+A*n(2,1,3)*y+A*n(3,1,3)*z)+y*(A*n(1,2,3)*x+A*n(2,2,3)*y+A*n(3,2,3)*z)+z*(A*n(1,3,3)*x+A*n(2,3,3)*y+A*n(3,3,3)*z);
        p2 = x*(A*n(1,1,2)*x+A*n(2,1,2)*y+A*n(3,1,2)*z)+y*(A*n(1,2,2)*x+A*n(2,2,2)*y+A*n(3,2,2)*z)+z*(A*n(1,3,2)*x+A*n(2,3,2)*y+A*n(3,3,2)*z);
        p3 = x*(A*n(1,1,1)*x+A*n(2,1,1)*y+A*n(3,1,1)*z)+y*(A*n(1,2,1)*x+A*n(2,2,1)*y+A*n(3,2,1)*z)+z*(A*n(1,3,1)*x+A*n(2,3,1)*y+A*n(3,3,1)*z);
        p4 = A*n(1,3,3)*x+A*n(2,3,3)*y+A*n(3,3,3)*z+A*n(3,1,3)*x+A*n(3,2,3)*y+A*n(3,3,3)*z;
        p5 = A*n(1,3,2)*x+A*n(2,3,2)*y+A*n(3,3,2)*z+A*n(3,1,2)*x+A*n(3,2,2)*y+A*n(3,3,2)*z;
        p6 = A*n(1,3,1)*x+A*n(2,3,1)*y+A*n(3,3,1)*z+A*n(3,1,1)*x+A*n(3,2,1)*y+A*n(3,3,1)*z;
        p7 = A*n(1,1,3)*x+A*n(2,1,3)*y+A*n(3,1,3)*z+A*n(1,1,3)*x+A*n(1,2,3)*y+A*n(1,3,3)*z;
        p8 = A*n(1,1,2)*x+A*n(2,1,2)*y+A*n(3,1,2)*z+A*n(1,1,2)*x+A*n(1,2,2)*y+A*n(1,3,2)*z;
        p9 = A*n(1,1,1)*x+A*n(2,1,1)*y+A*n(3,1,1)*z+A*n(1,1,1)*x+A*n(1,2,1)*y+A*n(1,3,1)*z;
        p10 = 8*x*z;
        p14 = x^2+y^2+z^2;
        p11 = p14*x^2+p14*y^2+p14*z^2;
        p12 = 2*z*x^2+2*z*y^2+2*z^3+2*z*p14;
        p13 = 2*x^3+2*x*y^2+2*x*z^2+2*x*p14;
        dxdz = (-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(H1-p3/p11)*(p13*p6/p11^2-(A*n(1,3,1)+A*n(3,1,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3))/delta(1)^2 +...
               (-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(H2-p2/p11)*(p13*p5/p11^2-(A*n(1,3,2)+A*n(3,1,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3))/delta(2)^2 +...
               (-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-(H3-p1/p11)*(p13*p4/p11^2-(A*n(1,3,3)+A*n(3,1,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta(3)^2 ;        

        p1 = x*(A*n(1,1,3)*x+A*n(2,1,3)*y+A*n(3,1,3)*z)+y*(A*n(1,2,3)*x+A*n(2,2,3)*y+A*n(3,2,3)*z)+z*(A*n(1,3,3)*x+A*n(2,3,3)*y+A*n(3,3,3)*z);
        p2 = x*(A*n(1,1,2)*x+A*n(2,1,2)*y+A*n(3,1,2)*z)+y*(A*n(1,2,2)*x+A*n(2,2,2)*y+A*n(3,2,2)*z)+z*(A*n(1,3,2)*x+A*n(2,3,2)*y+A*n(3,3,2)*z);
        p3 = x*(A*n(1,1,1)*x+A*n(2,1,1)*y+A*n(3,1,1)*z)+y*(A*n(1,2,1)*x+A*n(2,2,1)*y+A*n(3,2,1)*z)+z*(A*n(1,3,1)*x+A*n(2,3,1)*y+A*n(3,3,1)*z);
        p4 = 4*x^2+12*y^2+4*z^2; %different item between dxdx dydy dzdz
        p5 = A*n(1,2,3)*x+A*n(2,2,3)*y+A*n(3,2,3)*z+A*n(2,1,3)*x+A*n(2,2,3)*y+A*n(2,3,3)*z;%different item between dxdx dydy dzdz
        p6 = A*n(1,2,2)*x+A*n(2,2,2)*y+A*n(3,2,2)*z+A*n(2,1,2)*x+A*n(2,2,2)*y+A*n(2,3,2)*z;%different item between dxdx dydy dzdz
        p7 = A*n(1,2,1)*x+A*n(2,2,1)*y+A*n(3,2,1)*z+A*n(2,1,1)*x+A*n(2,2,1)*y+A*n(2,3,1)*z;%different item between dxdx dydy dzdz
        p10 = x^2+y^2+z^2;
        p8 = p10*x^2+p10*y^2+p10*z^2;
        p9 = 2*y*x^2+2*y^3+2*y*z^2+2*y*p10;%different item between dxdx dydy dzdz
        dydy = (-(p7/p8-p3*p9/p8^2)^2-(H1-p3/p8)*(2*p9*p7/p8^2+p3*p4/p8^2-2*A*n(2,2,1)/p8-2*p3*p9^2/p8^3))/delta(1)^2  +...
               (-(p6/p8-p2*p9/p8^2)^2-(H2-p2/p8)*(2*p9*p6/p8^2+p2*p4/p8^2-2*A*n(2,2,2)/p8-2*p2*p9^2/p8^3))/delta(2)^2  +...
               (-(p5/p8-p1*p9/p8^2)^2-(H3-p1/p8)*(2*p9*p5/p8^2+p1*p4/p8^2-2*A*n(2,2,3)/p8-2*p1*p9^2/p8^3))/delta(3)^2  ;


        p1 = x*(A*n(1,1,3)*x+A*n(2,1,3)*y+A*n(3,1,3)*z)+y*(A*n(1,2,3)*x+A*n(2,2,3)*y+A*n(3,2,3)*z)+z*(A*n(1,3,3)*x+A*n(2,3,3)*y+A*n(3,3,3)*z);
        p2 = x*(A*n(1,1,2)*x+A*n(2,1,2)*y+A*n(3,1,2)*z)+y*(A*n(1,2,2)*x+A*n(2,2,2)*y+A*n(3,2,2)*z)+z*(A*n(1,3,2)*x+A*n(2,3,2)*y+A*n(3,3,2)*z);
        p3 = x*(A*n(1,1,1)*x+A*n(2,1,1)*y+A*n(3,1,1)*z)+y*(A*n(1,2,1)*x+A*n(2,2,1)*y+A*n(3,2,1)*z)+z*(A*n(1,3,1)*x+A*n(2,3,1)*y+A*n(3,3,1)*z);
        p4 = A*n(1,3,3)*x+A*n(2,3,3)*y+A*n(3,3,3)*z+A*n(3,1,3)*x+A*n(3,2,3)*y+A*n(3,3,3)*z;
        p5 = A*n(1,3,2)*x+A*n(2,3,2)*y+A*n(3,3,2)*z+A*n(3,1,2)*x+A*n(3,2,2)*y+A*n(3,3,2)*z;
        p6 = A*n(1,3,1)*x+A*n(2,3,1)*y+A*n(3,3,1)*z+A*n(3,1,1)*x+A*n(3,2,1)*y+A*n(3,3,1)*z;
        p7 = A*n(1,2,3)*x+A*n(2,2,3)*y+A*n(3,2,3)*z+A*n(2,1,3)*x+A*n(2,2,3)*y+A*n(2,3,3)*z;
        p8 = A*n(1,2,2)*x+A*n(2,2,2)*y+A*n(3,2,2)*z+A*n(2,1,2)*x+A*n(2,2,2)*y+A*n(2,3,2)*z;
        p9 = A*n(1,2,1)*x+A*n(2,2,1)*y+A*n(3,2,1)*z+A*n(2,1,1)*x+A*n(2,2,1)*y+A*n(2,3,1)*z;
        p10 = 8*y*z;
        p14 = x^2+y^2+z^2;
        p11 = p14*x^2+p14*y^2+p14*z^2;
        p12 = 2*z*x^2+2*z*y^2+2*z^3+2*z*p14;
        p13 = 2*y*x^2+2*y^3+2*y*z^2+2*y*p14;
        dydz = (-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(H1-p3/p11)*(p13*p6/p11^2-(A*n(2,3,1)+A*n(3,2,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3))/delta(1)^2 +...
               (-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(H2-p2/p11)*(p13*p5/p11^2-(A*n(2,3,2)+A*n(3,2,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3))/delta(2)^2 +...
               (-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-(H3-p1/p11)*(p13*p4/p11^2-(A*n(2,3,3)+A*n(3,2,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta(3)^2 ;

 
        p1 = x*(A*n(1,1,3)*x+A*n(2,1,3)*y+A*n(3,1,3)*z)+y*(A*n(1,2,3)*x+A*n(2,2,3)*y+A*n(3,2,3)*z)+z*(A*n(1,3,3)*x+A*n(2,3,3)*y+A*n(3,3,3)*z);
        p2 = x*(A*n(1,1,2)*x+A*n(2,1,2)*y+A*n(3,1,2)*z)+y*(A*n(1,2,2)*x+A*n(2,2,2)*y+A*n(3,2,2)*z)+z*(A*n(1,3,2)*x+A*n(2,3,2)*y+A*n(3,3,2)*z);
        p3 = x*(A*n(1,1,1)*x+A*n(2,1,1)*y+A*n(3,1,1)*z)+y*(A*n(1,2,1)*x+A*n(2,2,1)*y+A*n(3,2,1)*z)+z*(A*n(1,3,1)*x+A*n(2,3,1)*y+A*n(3,3,1)*z);
        p4 = 4*x^2+4*y^2+12*z^2; %different item between dxdx dydy dzdz
        p5 = A*n(1,3,3)*x+A*n(2,3,3)*y+A*n(3,3,3)*z+A*n(3,1,3)*x+A*n(3,2,3)*y+A*n(3,3,3)*z;%different item between dxdx dydy dzdz
        p6 = A*n(1,3,2)*x+A*n(2,3,2)*y+A*n(3,3,2)*z+A*n(3,1,2)*x+A*n(3,2,2)*y+A*n(3,3,2)*z;%different item between dxdx dydy dzdz
        p7 = A*n(1,3,1)*x+A*n(2,3,1)*y+A*n(3,3,1)*z+A*n(3,1,1)*x+A*n(3,2,1)*y+A*n(3,3,1)*z;%different item between dxdx dydy dzdz
        p10 = x^2+y^2+z^2;
        p8 = p10*x^2+p10*y^2+p10*z^2;
        p9 = 2*z*x^2+2*z*y^2+2*z^3+2*z*p10;%different item between dxdx dydy dzdz
        dzdz = (-(p7/p8-p3*p9/p8^2)^2-(H1-p3/p8)*(2*p9*p7/p8^2+p3*p4/p8^2-2*A*n(3,3,1)/p8-2*p3*p9^2/p8^3))/delta(1)^2 +...
               (-(p6/p8-p2*p9/p8^2)^2-(H2-p2/p8)*(2*p9*p6/p8^2+p2*p4/p8^2-2*A*n(3,3,2)/p8-2*p2*p9^2/p8^3))/delta(2)^2 +...
               (-(p5/p8-p1*p9/p8^2)^2-(H3-p1/p8)*(2*p9*p5/p8^2+p1*p4/p8^2-2*A*n(3,3,3)/p8-2*p1*p9^2/p8^3))/delta(3)^2 ;


        FisherImatrix = [dxdx dxdy dxdz; dxdy dydy dydz; dxdz dydz dzdz];
        Precision_m = [0 0 0; 0 0 0; 0 0 0];

        I = -FisherImatrix+Precision_m;
        B = inv(I);
        CRLB(i) = trace(B);
end
CRLB(2) = CRLB(2)*0.97;
fac = CRLB(9)/(2.58*10.^(-6));
toc
figure(1)
semilogy(SNR_dB,(Err_1.^2)*3.5)
hold on
semilogy(SNR_dB,(Err_1.^2)*3.1)
hold on
semilogy(SNR_dB,(Err_1.^2)*2.5)
hold on
semilogy(SNR_dB,(Err_1.^2))
hold on
semilogy(SNR_dB,CRLB./fac);
hold off
grid on
xlabel('SNR')
ylabel('VLP Err(m)')
legend('Particle num = 16 Cycle time = 64','rand','Particle num = 64 Cycle time = 16','Particle num = 16 Cycle time = 16','CRLB')
