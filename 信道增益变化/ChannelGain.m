%*********************************************************************************************%
% Assume the size of Room is 10m * 10m *4m and the only LED in the ceil of the center of room %
% We define the lower left corner as the origin point (0, 0, 0) and the                       %
% other three point is (0, 10, 0), (10, 0, 0), (10, 10, 0), the coordinate                    %
% of the LED in this Coordinate system is (5, 5, 4). The Normal Vector of                     %
% LED is constant (0, 0, -1) in our consider(in most case this is                             %
% reasonable because the fixed LED installation Method), and the Normal                       %
% Vector of PD is random because of the rotation and moving, But we think                     %
% we can get this information throught the internal sensor of the phone                       %
%*********************************************************************************************%
%Summary                                                                                      %
%Size of the room 10m * 10m *4m                                                               %
%Coordinate of LED (5, 5, 4)                                                                  %
%Normal Vector of LED (0, 0, -1)                                                              %
%*********************************************************************************************%
tic
clear clc

AR    = 1; %Recieve are of pd (unit:cm^2)
P_Ti  = 1;%Transmit power of led (unit:unknown)
X_led = [0  0 -1];
X_pd  = [sqrt(2)/4  sqrt(2)/4  sqrt(3)/2];
P_led = [1.5  1.5  3];

d = 0.5:0.25:3;
P_Theta_Psi = zeros(1,length(d));
for i = 1:length(d)
    P_pd  = [1.5  1.5  P_led(3)-d(i)];
    D_led = P_pd  - P_led;
    D_pd  = P_led - P_pd;
    Theta = dot(D_led,X_led)/(norm(D_led,2)*norm(X_led,2)); %Irradiance Angle(Cos value)
    Psi   = dot(D_pd,X_pd)/(norm(D_pd,2)*norm(X_pd,2)); %Incidence Angle
    Self_Power_Angle_led = 60; 
    Self_Power_Angle_pd  = 60;
    m = -log(2) /log(cos(Self_Power_Angle_led/180*pi)); %Lambertian Order of led
    M = -log(2) /log(cos(Self_Power_Angle_pd/180*pi));  %Lambertian Order of pd
    P_Theta_Psi(i) = (m+1)*AR*P_Ti*(Theta^m*Psi^M)/(2*pi*(norm(D_led,2)^2));    
end
plot(d,10*log(P_Theta_Psi)/log(10));
hold on
plot(d,10*log(P_Theta_Psi*2)/log(10));
hold on
plot(d,10*log(P_Theta_Psi*3)/log(10));

legend('Without Rotation','2 Rotation states','3 Rotation states')



