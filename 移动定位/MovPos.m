% 场景：
%   在一个室内空间范围内，一个机器人在初始位置开始移动，旋转平台旋转/不旋转的情况下
%   移动,将实际移动情况与定位结果显示出来。
% 关键参数：
%   空间大小：9m*9m*3m
%   机器人初始位置：P_ue_init
%   机器人移动速度：V_ue
%   机器人旋转平台角速度：W_ue/0
%   机器人光电检测器有效接受面积:A_rd
%   光源的位置：P_leds
%   光源的伦勃朗辐照阶数：m_Lam
%   光源的辐照半功率角:theta_LED
% 步骤：
%   1.创建机器人二维移动过程
%   2.在单位时间内，旋转平台不动的情况下完成K=3的VLP定位
%   3.在单位时间内，旋转平台转动的情况下完成K=3的VLP定位