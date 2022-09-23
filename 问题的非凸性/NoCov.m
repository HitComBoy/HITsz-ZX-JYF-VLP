%*********************************************************************************************%
% Calculate the Carmer-Rao Bound of Estimation of Position(x,y,z), We consider the Transform  %
% Matrix H related with Facing Vector of PD, Lambertian Order, Parameter A =(m+1)*Ar*Pti/2pi  %
% and the Position(x,y,z) itself. So we need to calculate the relation between CRLB and these %
% parameters.                                                                                 %
%*********************************************************************************************%
%Function Find_Facing_Vector: Finding the facing Vector of different position                 %
%Function Find_LinearTransform_Matrix: Calculate the H for different position and its related %
%Facing Vector, different Parameter A.                                                        %  
%Size of Place:3*3*3                                                                          %
%*********************************************************************************************%


clear clc
step_len = 0.01;
z = 0.5;
m = 1;
M = 1;
ArP = 10;
N_led = [0 0 -1];
P_led = [1.5 1.5 3];
P_pd = [0 0 0.5];
N_pd = Facing_Vector(P_pd, P_led);
H1 = Theory_Power(P_pd, P_led, N_pd(1,:), N_led, m, M, ArP);
H2 = Theory_Power(P_pd, P_led, N_pd(2,:), N_led, m, M, ArP);
H3 = Theory_Power(P_pd, P_led, N_pd(3,:), N_led, m, M, ArP);
SNR = -10;
Var_Noise = mean([H1 H2 H3])/10^(SNR/10);
C1 = H1+sqrt(Var_Noise)*randn(1); 
C2 = H2+sqrt(Var_Noise)*randn(1); 
C3 = H3+sqrt(Var_Noise)*randn(1); 
Err_result=zeros(3/step_len+1,3/step_len+1);
for x = 0:step_len:3
    for y = 0:step_len:3
        T1 =  Theory_Power([x y 0.5], P_led, N_pd(1,:), N_led, m, M, ArP);
        T2 =  Theory_Power([x y 0.5], P_led, N_pd(2,:), N_led, m, M, ArP);
        T3 =  Theory_Power([x y 0.5], P_led, N_pd(3,:), N_led, m, M, ArP);
        v = [T1 T2 T3]-[C1 C2 C3];
        err = norm(v)^2;
        Err_result(round(x/step_len+1),round(y/step_len+1))=err;
    end
end
mesh(Err_result)
xticks([0 100 200 300])
yticks([0 100 200 300])
xticklabels({'0','1','2','3'})
yticklabels({'0','1','2','3'})
xlabel('X_coordinate(m)')
ylabel('y_coordinate(m)')
zlabel('Square Error')
title('Solution Space of the nonconvex function')


