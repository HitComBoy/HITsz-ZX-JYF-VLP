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
% P_pd = [0 0 0];
% P_led = [5 5 3];
% Fvector = Facing_Vector(P_pd , P_led)
% quiver3(P_pd(1),P_pd(2),P_pd(3),Fvector(1,1),Fvector(1,2),Fvector(1,3))
% hold on
% quiver3(P_pd(1),P_pd(2),P_pd(3),Fvector(2,1),Fvector(2,2),Fvector(2,3))
% hold on
% quiver3(P_pd(1),P_pd(2),P_pd(3),Fvector(3,1),Fvector(3,2),Fvector(3,3))
% hold on
% quiver3(P_pd(1),P_pd(2),P_pd(3),P_led(1)-P_pd(1),P_led(2)-P_pd(2),P_led(3)-P_pd(3),'LineWidth',0.75)
% axis equal
clear clc
step_len = 0.1;
z = 0.5;
m = 1;
M = 1;
ArP = 10;
N_led = [0 0 -1];
P_led = [1.5 1.5 3];
P_pd_receive_max =[1.5 1.5 0.5];
SNR = 100;
Var_Noise = ArP/10^(SNR/10);


%Var_Noise = Theory_Power(P_pd_receive_max, P_led, P_led-P_pd_receive_max, N_led, m, M, ArP)/10^(SNR/10); %Maximum SNR in the room
CRLB = zeros(3/step_len+1,3/step_len+1,3);
SNR_everypoint = zeros(3/step_len+1,3/step_len+1);
Power_1 = zeros(3/step_len+1,3/step_len+1);
Power_2 = zeros(3/step_len+1,3/step_len+1);
for x = 0:step_len:3
    for y = 0:step_len:3
        Position = [x y z];
        N_pd = Facing_Vector(Position, P_led);
        H = LinearTransform_Matrix(Position, P_led, N_pd, N_led, m, M, ArP);
        %Var_Noise = Theory_Power([x y 0.5], P_led, N_pd, N_led, m, M, ArP)/10^(SNR/10); 
        Mid_CRLB = Var_Noise*inv(transpose(H)*H);
        CRLB(round((x+step_len)/step_len),round((y+step_len)/step_len),1) = Mid_CRLB(1,1);
        CRLB(round((x+step_len)/step_len),round((y+step_len)/step_len),2) = Mid_CRLB(2,2);
        CRLB(round((x+step_len)/step_len),round((y+step_len)/step_len),3) = Mid_CRLB(3,3);
        SNR_everypoint(round((x+step_len)/step_len),round((y+step_len)/step_len)) = 10*log(Theory_Power(Position, P_led, N_pd, N_led, m, M, ArP)...
        /Var_Noise)/log(10);
        Power_1(round((x+step_len)/step_len),round((y+step_len)/step_len)) = Theory_Power(Position, P_led, N_pd, N_led, m, M, ArP);
        Power_2(round((x+step_len)/step_len),round((y+step_len)/step_len)) = function_Power(Position, P_led, N_pd, N_led, m, M, ArP);
%       Err = Power_1-Power_2;
    end
end
mesh(CRLB(:,:,1))
xticks([0 10 20 30])
yticks([0 10 20 30])
xticklabels({'0','1','2','3'})
yticklabels({'0','1','2','3'})
xlabel('X_coordinate(m)')
ylabel('y_coordinate(m)')
zlabel('MSE')
title('In the case of height = 0.5m, transmitted power = 15w')

figure
mesh(SNR_everypoint)
xticks([0 10 20 30])
yticks([0 10 20 30])
xticklabels({'0','1','2','3'})
yticklabels({'0','1','2','3'})
xlabel('X_coordinate(m)')
ylabel('y_coordinate(m)')
zlabel('SNR')
title('In the case of height = 0.5m, transmitted power = 15w')

figure
mesh(Power_1)
xticks([0 10 20 30])
yticks([0 10 20 30])
xticklabels({'0','1','2','3'})
yticklabels({'0','1','2','3'})
xlabel('X_coordinate(m)')
ylabel('y_coordinate(m)')
zlabel('Receive Power')
title('In the case of height = 0.5m, transmitted power = 15w')
% figure
% mesh(Power_2)
%acos(dot(D_led,X_led)/(norm(D_led,2)*norm(X_led,2)))/pi*180