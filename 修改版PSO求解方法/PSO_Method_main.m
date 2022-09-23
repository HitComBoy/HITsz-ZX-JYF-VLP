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
K=3; %旋转状态数
test_time = 2000;     %在每个坐标下，定位误差测试次数（最后取平均）
Pos(1,:) = 0:0.2:3; %生成测试点的x坐标
Pos(2,:) = 0:0.2:3; %生成测试点的y坐标
Pos(3,:) = 0.5;     %生成测试点的z坐标(为了便于展示，暂时固定z坐标)

Err_1 = zeros(size(Pos,2),size(Pos,2)); %在每个坐标下de定位误差(128 100)
Err_2 = zeros(size(Pos,2),size(Pos,2)); %在每个坐标下de定位误差(64 100)
MidErr =zeros(1,test_time);             %纪录每次测试下的误差

num = 64;        %粒子数量
num_cycle = 40; %粒子群循环次数
for i=1:size(Pos,2)
    for j=1:size(Pos,2)
        P_pd = [Pos(1,i) Pos(2,j) Pos(3,1)]; %生成测试点
        N_pd = Facing_Vector(P_pd, P_led, K);   %PD旋转过程中的法线向量
        parfor k=1:test_time
            Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);                         %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
            Var = mean(Power)/10000;
            Power = Power+sqrt(Var).*randn(1,3); 
            Estimated_Pos = PSO_Method(Power, P_led, N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置
            Err = norm((P_pd - Estimated_Pos), 2);                                            %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                  %累加，最后求平均
        end
        Err_1(i,j) = mean(MidErr); %在该测试点下，平均定位误差
    end
    ['Position(1/2),remain' num2str(size(Pos,2)-i)]    
end

Pos(3,:) = 2;     %生成测试点的z坐标(为了便于展示，暂时固定z坐标)
for i=1:size(Pos,2)
    for j=1:size(Pos,2)
        P_pd = [Pos(1,i) Pos(2,j) Pos(3,1)]; %生成测试点
        N_pd = Facing_Vector(P_pd, P_led, K);   %PD旋转过程中的法线向量
        parfor k=1:test_time
            Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);                         %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
            Var = mean(Power)/10000;
            Power = Power+sqrt(Var).*randn(1,3); 
            Estimated_Pos = PSO_Method(Power, P_led, N_pd, N_led, m, M, Ar, num, num_cycle); %在该测试点下，算法所得到的估计位置
            Err = norm((P_pd - Estimated_Pos), 2);                                            %在该测试点下，该次定位误差
            MidErr(k) = Err;                                                                  %累加，最后求平均
        end
        Err_2(i,j) = mean(MidErr); %在该测试点下，平均定位误差
    end   
    ['Position(2/2),remain' num2str(size(Pos,2)-i)]
end
figure(1)
mesh(Pos(1,:),Pos(2,:),Err_1,'edgecolor','#77AC30')
hold on
mesh(Pos(1,:),Pos(2,:),Err_2,'edgecolor','#A2142F')
hold off
grid on
xlabel('X position')
ylabel('Y position')
zlabel('Localization error (m)')
title('The relation ship between position and position accuracy')
legend('Height=0.5','Height=2')

% %%%%%%%*For the num of partices, find the relation ship between num and position accuracy*%%%%%
% num = 10:10:80;   %粒子数量
% num_cycle = 80; %粒子群循环次数
% P_pd = [1.5 1.5 0.5];               %设置一个固定的样本点
% N_pd = Facing_Vector(P_pd, P_led);  %PD旋转过程中的法线向量
% Err_1 = zeros(1,size(num,2));       %在每个坐标下de定位误差(移动100次)
% Err_2 = zeros(1,size(num,2));       %在每个坐标下de定位误差(移动50次）
% Err_3 = zeros(1,size(num,2));       %在每个坐标下de定位误差(移动25次）
% MidErr =zeros(1,test_time);         %纪录每次测试下的误差
% for i=1:size(num,2)
%         parfor k=1:test_time
%             Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);                            %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
%             Var = mean(Power)/10000;
%             Power = Power+sqrt(Var).*randn(1,3);   
%             Estimated_Pos = PSO_Method(Power, P_led, N_pd, N_led, m, M, Ar, num(i), num_cycle); %在该测试点下，算法所得到的估计位置
%             Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
%             MidErr(k) = Err;                                                                     %累加，最后求平均
%         end
%         Err_1(i) = mean(MidErr); %在该测试点下，平均定位误差
%         ['Particle(1/3),remain' num2str(size(num,2)-i)]
% end
% 
% num_cycle = 40; %改变粒子群循环次数
% for i=1:size(num,2)
%         parfor k=1:test_time
%             Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);                            %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
%             Var = mean(Power)/10000;
%             Power = Power+sqrt(Var).*randn(1,3);                          %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
%             Estimated_Pos = PSO_Method(Power, P_led, N_pd, N_led, m, M, Ar, num(i), num_cycle); %在该测试点下，算法所得到的估计位置
%             Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
%             MidErr(k) = Err;                                                                     %累加，最后求平均
%         end
%         Err_2(i) = mean(MidErr); %在该测试点下，平均定位误差
%         ['Particle(2/3),remain' num2str(size(num,2)-i)]
% end
% 
% num_cycle = 20; %改变粒子群循环次数
% for i=1:size(num,2)
%         parfor k=1:test_time
%             Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);                            %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
%             Var = mean(Power)/10000;
%             Power = Power+sqrt(Var).*randn(1,3);                            %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
%             Estimated_Pos = PSO_Method(Power, P_led, N_pd, N_led, m, M, Ar, num(i), num_cycle); %在该测试点下，算法所得到的估计位置
%             Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
%             MidErr(k) = Err;                                                                     %累加，最后求平均
%         end
%         Err_3(i) = mean(MidErr); %在该测试点下，平均定位误差 
%         ['Particle(3/3),remain' num2str(size(num,2)-i)]
% end
% 
% figure(2)
% semilogy(num,Err_1)
% hold on
% semilogy(num,Err_2)
% hold on
% semilogy(num,Err_3)
% hold off
% grid on
% xlabel('The num of particles')
% ylabel('MSE')
% title('The relation ship between particle num and position accuracy')
% legend('Cycle time = 100','Cycle time = 50','Cycle time = 25')
% 
% %********For the num of Cycle, find the relation ship between num and position accuracy*****%
% num_cycle = 10:10:80; %粒子群循环次数
% P_pd = [1.5 1.5 0.5];               %设置一个固定的样本点
% N_pd = Facing_Vector(P_pd, P_led);  %PD旋转过程中的法线向量
% Err_1 = zeros(1,size(num_cycle,2));       %在每个坐标下de定位误差(粒子数128)
% Err_2 = zeros(1,size(num_cycle,2));       %在每个坐标下de定位误差(粒子数64）
% Err_3 = zeros(1,size(num_cycle,2));       %在每个坐标下de定位误差(粒子数32）
% MidErr =zeros(1,test_time);         %纪录每次测试下的误差
% 
% num = 80;   %粒子数量
% for i=1:size(num_cycle,2)
%         parfor k=1:test_time
%             Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);                            %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
%             Var = mean(Power)/10000;
%             Power = Power+sqrt(Var).*randn(1,3);                            %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
%             Estimated_Pos = PSO_Method(Power, P_led, N_pd, N_led, m, M, Ar, num, num_cycle(i)); %在该测试点下，算法所得到的估计位置
%             Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
%             MidErr(k) = Err;                                                                     %累加，最后求平均
%         end
%         Err_1(i) = mean(MidErr); %在该测试点下，平均定位误差
%         ['Cycle(1/3),remain' num2str(size(num_cycle,2)-i)]
% end
% 
% num = 40;   %粒子数量
% for i=1:size(num_cycle,2)
%         parfor k=1:test_time
%             Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);                            %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
%             Var = mean(Power)/10000;
%             Power = Power+sqrt(Var).*randn(1,3);                           %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
%             Estimated_Pos = PSO_Method(Power, P_led, N_pd, N_led, m, M, Ar, num, num_cycle(i)); %在该测试点下，算法所得到的估计位置
%             Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
%             MidErr(k) = Err;                                                                     %累加，最后求平均
%         end
%         Err_2(i) = mean(MidErr); %在该测试点下，平均定位误差
%         ['Cycle(2/3),remain' num2str(size(num_cycle,2)-i)]
% end
% 
% num = 20;   %粒子数量
% for i=1:size(num_cycle,2)
%         parfor k=1:test_time
%             Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar);                            %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
%             Var = mean(Power)/10000;
%             Power = Power+sqrt(Var).*randn(1,3);                            %在该测试点下，PD旋转过程中接收到的功率（无噪情况）
%             Estimated_Pos = PSO_Method(Power, P_led, N_pd, N_led, m, M, Ar, num, num_cycle(i)); %在该测试点下，算法所得到的估计位置
%             Err = norm((P_pd - Estimated_Pos), 2);                                               %在该测试点下，该次定位误差
%             MidErr(k) = Err;                                                                     %累加，最后求平均
%         end
%         Err_3(i) = mean(MidErr); %在该测试点下，平均定位误差
%         ['Cycle(3/3),remain' num2str(size(num_cycle,2)-i)]
% end
% 
% figure(3)
% semilogy(num_cycle,Err_1)
% hold on
% semilogy(num_cycle,Err_2)
% hold on
% semilogy(num_cycle,Err_3)
% hold off
% grid on
% xlabel('The times of Cycle')
% ylabel('MSE')
% title('The relation ship between Cycle times and position accuracy')
% legend('Particle = 128','Particle = 64','Particle = 32')



toc