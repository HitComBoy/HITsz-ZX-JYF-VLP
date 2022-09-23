% 场景：
%   在一个室内空间范围内，一个机器人在初始位置开始移动，旋转平台旋转/不旋转的情况下
%   移动,将实际移动情况与定位结果显示出来。
%
% 关键参数：
%   空间大小：space_Size 9m*9m*3m
%
%   光源的伦勃朗辐照阶数：m_Lam
%   光源的位置：P_leds
%
%   机器人初始位置：P_ue_init
%   机器人移动速度：V_ue
%   机器人旋转平台角速度：W_ue/0
%   机器人光电检测器有效接受面积:A_rd
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
P_ue_init = [1,1,1];
V_ue = [0.5,0.5,0];
W_ue = pi;
A_rd = 1;
%*****************************************************
%                  移动展示
%*****************************************************

P_ue = P_ue_init;
figure
plot(P_ue(1),P_ue(2),'*','Color','#A1111F','MarkerSize',12);
axis([0 9 0 9]);
hold on;
for i=1:13
   P_ue = P_ue + V_ue;
   plot(P_ue(1),P_ue(2),'.','Color','#A2142F','MarkerSize',10);
   hold on;
   pause(0.2);
end