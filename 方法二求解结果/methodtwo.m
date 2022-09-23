function Estimated_Pos = methodtwo(Power_noise, P_led, N_pd, z)
%***********************************************************************************%
%Using the method2 to caculate the position result(Known Z)                         % 
%***********************************************************************************%
num_formula = size(N_pd,1)-1; %旋转方程组中等式的个数
a = zeros(num_formula,3); %旋转方程组的参数矩阵
K = zeros(1,num_formula);     %不同状态之间信道的相除值
for i=1:num_formula
    K(i) = Power_noise(i)/Power_noise(i+1);
    a(i,1) = K(i)*N_pd((i+1),1) - N_pd(i,1);
    a(i,2) = K(i)*N_pd((i+1),2) - N_pd(i,2);
    a(i,3) = K(i)*N_pd((i+1),3) - N_pd(i,3);    
end

if rank(a) == 3
    Estimated_Pos = [0 0 0]';
else
    Estimated_Pos = null(a); %得出最终的定位结果
    Estimated_Pos
    Estimated_Pos = Estimated_Pos/(Estimated_Pos(3)/(z-P_led(3))); %消除差向量的长度不确定性（利用已知坐标Z）
    Estimated_Pos = Estimated_Pos + P_led'; %得出最终的定位结果
    Estimated_Pos
% end    
end

