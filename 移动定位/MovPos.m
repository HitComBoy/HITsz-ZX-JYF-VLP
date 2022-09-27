% 场景：
%   在一个室内空间范围内，一个机器人在初始位置开始移动，旋转平台旋转/不旋转的情况下
%   移动,将实际移动情况与定位结果显示出来。
%
% 关键参数：
%   空间大小：space_Size 9m*9m*3m
%
%   光源的伦勃朗辐照阶数：m_Lam
%   光源的位置：P_leds
%   光源的法线向量：Q_led
%   光源的半功率角：FOV
%
%   机器人初始位置：P_ue_init
%   机器人移动速度：V_ue
%   机器人旋转平台角速度：W_ue/0
%   机器人光电检测器有效接受面积:A_rd
%   机器人光电检测的伦勃朗阶数：M_Lam
%
% 步骤：
%   1.创建机器人二维移动过程
%   2.在单位时间内，旋转平台不动的情况下完成K=3的VLP定位
%   3.在单位时间内，旋转平台转动的情况下完成K=3的VLP定位

%*****************************************************
%                  参数初始化
%*****************************************************
space_Size = [9,9,3];
m_Lam = 1;
P_leds = [3,3,3;3,6,3;6,3,3;6,6,3];
Q_led = [0,0,-1];
FOV = cos(pi/3);
P_ue_init = [1,1,1];
V_ue = [0.2,0.2,0];
W_ue = pi;
A_rd = 1;
M_Lam = 1;
%*****************************************************
%                  1.机器人的二位移动过程
%*****************************************************
% 
% P_ue = P_ue_init;
% figure
% plot(P_ue(1),P_ue(2),'*','Color','#A1111F','MarkerSize',12);
% axis([0 9 0 9]);
% hold on;
% for i=1:13
%    P_ue = P_ue + V_ue;
%    plot(P_ue(1),P_ue(2),'.','Color','#A2142F','MarkerSize',10);
%    hold on;
%    pause(0.2);
% end
%*****************************************************
%  2.在单位时间内，旋转平台不动的情况下完成K=3的VLP定位
%*****************************************************

P_ue = P_ue_init;
figure
plot(P_leds(1,1),P_leds(1,2),'s','Color','r','MarkerSize',12);
hold on;
rectangle('Position',[P_leds(1,1)-(2*sqrt(3)) P_leds(1,2)-(2*sqrt(3)) 2*2*sqrt(3) 2*2*sqrt(3)],'Curvature',[1 1],'EdgeColor','r');
axis equal
hold on;
plot(P_leds(2,1),P_leds(2,2),'s','Color','g','MarkerSize',12);
hold on;
rectangle('Position',[P_leds(2,1)-(2*sqrt(3)) P_leds(2,2)-(2*sqrt(3)) 2*2*sqrt(3) 2*2*sqrt(3)],'Curvature',[1 1],'EdgeColor','g');
axis equal
hold on;
plot(P_leds(3,1),P_leds(3,2),'s','Color','b','MarkerSize',12);
hold on;
rectangle('Position',[P_leds(3,1)-(2*sqrt(3)) P_leds(3,2)-(2*sqrt(3)) 2*2*sqrt(3) 2*2*sqrt(3)],'Curvature',[1 1],'EdgeColor','b');
axis equal
hold on;
plot(P_leds(4,1),P_leds(4,2),'s','Color','m','MarkerSize',12);
hold on;
rectangle('Position',[P_leds(4,1)-(2*sqrt(3)) P_leds(4,2)-(2*sqrt(3)) 2*2*sqrt(3) 2*2*sqrt(3)],'Curvature',[1 1],'EdgeColor','m');
axis equal
hold on;
axis([0 9 0 9]);
hold on;
for i=1:10
   V_ue = [2,2,0];
   V_ue = V_ue.*[rand rand 1];
   P_ue = P_ue + V_ue;
   color = [rand rand rand];
   %plot([P_ue(1)-V_ue(1) P_ue(1)],[P_ue(2)-V_ue(2) P_ue(2)],'Color','r');
   plot(P_ue(1),P_ue(2),'.','Color',color,'MarkerSize',10);
   hold on;
   
   Q_pd = [0,0,1];                                                             %PD暂时在移动过程中不移动
   cover_LEDs = [0,0,0];                                                       %用cover_LEDs作为灯源库来存放可以cover的灯源
   for j=1:size(P_leds)                                                        %处理当前PD能被几个光源所覆盖
       r = P_ue-P_leds(j,:);                                                   %对于第j个灯源来说的入射向量
       cos_theta_j = Q_led*r'/(norm(Q_led,2)*norm(r,2));
       %theta_j = acos(cos_theta_j);                                            %求对于第j个灯源来说，当前PD位置的辐照角度
       if cos_theta_j >= FOV                                                       %如果处于第i个灯源的覆盖范围，则添加到灯源库cover_LEDs
           cover_LEDs = [cover_LEDs;P_leds(j,:)];
       end
   end
   cover_LEDs = cover_LEDs(2:size(cover_LEDs),:);                              %当前cover_LEDs包含了能覆盖当前位置的所有LED
   cover_Power = 0;                                                            %用cover_Power来存放每个LED对当前位置的理论接收功率
   for j=1:size(cover_LEDs)
       cover_Power = [cover_Power;Theory_Power(P_ue - V_ue, cover_LEDs(j,:), Q_pd, Q_led, m_Lam, M_Lam, A_rd)];     %上一个位置处的接收功率
       cover_Power = [cover_Power;Theory_Power(P_ue - 0.5*V_ue, cover_LEDs(j,:), Q_pd, Q_led, m_Lam, M_Lam, A_rd)]; %中间位置处的接收功率
       cover_Power = [cover_Power;Theory_Power(P_ue, cover_LEDs(j,:), Q_pd, Q_led, m_Lam, M_Lam, A_rd)];            %当前位置处的接收功率               
   end        
   cover_Power = cover_Power(2:size(cover_Power),:);                           %得到每个LED的理论功率
   Estimated_Pos = PSO_Method(cover_Power, cover_LEDs, Q_pd, Q_led, m_Lam, M_Lam, A_rd,V_ue); %在该测试点下，算法所得到的估计位置 
   plot(Estimated_Pos(1),Estimated_Pos(2),'*','Color',color,'MarkerSize',10);
   hold on;
   str =  ['This position is coverd by  ' num2str(size(cover_LEDs,1)) '  LED'];
   disp(str)
   str =  ['This time V_ue is = ' num2str(V_ue(1)) ' ' num2str(V_ue(2)) ' The Speed length is ' num2str(norm(V_ue,2))];
   disp(str)
   str =  ['This time error is = ' num2str(norm(Estimated_Pos-P_ue,2))];
   disp(str)
   pause(0.001);
end

       
        
