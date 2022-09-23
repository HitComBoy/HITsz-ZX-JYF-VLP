%*********************************************************************************************%
% Calculate the Carmer-Rao Bound of Estimation of Position(x,y,z), this new method was inspire%
% by the paper:Performance Limits of Visible Light-Based User Position and Orientation        %
% Estimation Using Received Signal Strength Under NLOS Propagation                            %
% Size of Place:3*3*3                                                                         %
%*********************************************************************************************%

%************This is the first step to calculate the expression of Fisher Matrix**************%
clear 
clc

%************Then set the basis parameter of actual situation and calculate value**************%
step_len = 0.1;
k = 0.5;

m = 1;
M = 1;
Ar = 1;
A = (m+1)*Ar/(2*pi);
test_point = [1.5 1.5 0.5];
N_led = [0 0 -1];
P_led = [1.5 1.5 3];
SNR = 0:10:60;

pi = 3.14159;
CRLB = zeros(length(SNR),1);
for i=1:length(SNR)

        Var_Noise = Theory_Power(test_point, P_led, P_led-test_point, N_led, m, M, Ar)/10^(SNR(i)/10);
        delta = sqrt(Var_Noise);
    
        x =  test_point(1) - P_led(1);
        y =  test_point(2) - P_led(2);
        z =  test_point(3) - P_led(3);
        N_pd = Facing_Vector(test_point, P_led);

        n(:,:,1) = -transpose(N_led)*N_pd(1,:);
        n(:,:,2) = -transpose(N_led)*N_pd(2,:);
        n(:,:,3) = -transpose(N_led)*N_pd(3,:);
 
        H1 = Theory_Power(test_point, P_led, N_pd(1,:), N_led, m, M, Ar);
        H2 = Theory_Power(test_point, P_led, N_pd(2,:), N_led, m, M, Ar);
        H3 = Theory_Power(test_point, P_led, N_pd(3,:), N_led, m, M, Ar);

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
        dxdx = (-(p7/p8-p3*p9/p8^2)^2-(p6/p8-p2*p9/p8^2)^2-(p5/p8-p1*p9/p8^2)^2-...
            (H1-p3/p8)*(2*p9*p7/p8^2+p3*p4/p8^2-2*A*n(1,1,1)/p8-2*p3*p9^2/p8^3)-...
            (H2-p2/p8)*(2*p9*p6/p8^2+p2*p4/p8^2-2*A*n(1,1,2)/p8-2*p2*p9^2/p8^3)-...
            (H3-p1/p8)*(2*p9*p5/p8^2+p1*p4/p8^2-2*A*n(1,1,3)/p8-2*p1*p9^2/p8^3))/delta^2;
 

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
        dxdy = (-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-...
            (H1-p3/p11)*(p13*p6/p11^2-(A*n(1,2,1)+A*n(2,1,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3)-...
            (H2-p2/p11)*(p13*p5/p11^2-(A*n(1,2,2)+A*n(2,1,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3)-...
            (H3-p1/p11)*(p13*p4/p11^2-(A*n(1,2,3)+A*n(2,1,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta^2;
 

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
        dxdz = (-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-...
            (H1-p3/p11)*(p13*p6/p11^2-(A*n(1,3,1)+A*n(3,1,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3)-...
            (H2-p2/p11)*(p13*p5/p11^2-(A*n(1,3,2)+A*n(3,1,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3)-...
            (H3-p1/p11)*(p13*p4/p11^2-(A*n(1,3,3)+A*n(3,1,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta^2;
 
        %Totally same as dxdy

        % dydx = (-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-...
        %     (H1-p3/p11)*(p13*p6/p11^2-(A*n(1,2,1)+A*n(2,1,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3)-...
        %     (H2-p2/p11)*(p13*p5/p11^2-(A*n(1,2,2)+A*n(2,1,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3)-...
        %     (H3-p1/p11)*(p13*p4/p11^2-(A*n(1,2,3)+A*n(2,1,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta^2;
 

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
        dydy = (-(p7/p8-p3*p9/p8^2)^2-(p6/p8-p2*p9/p8^2)^2-(p5/p8-p1*p9/p8^2)^2-...
            (H1-p3/p8)*(2*p9*p7/p8^2+p3*p4/p8^2-2*A*n(2,2,1)/p8-2*p3*p9^2/p8^3)-...
            (H2-p2/p8)*(2*p9*p6/p8^2+p2*p4/p8^2-2*A*n(2,2,2)/p8-2*p2*p9^2/p8^3)-...
            (H3-p1/p8)*(2*p9*p5/p8^2+p1*p4/p8^2-2*A*n(2,2,3)/p8-2*p1*p9^2/p8^3))/delta^2;
 

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
        dydz = (-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-...
            (H1-p3/p11)*(p13*p6/p11^2-(A*n(2,3,1)+A*n(3,2,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3)-...
            (H2-p2/p11)*(p13*p5/p11^2-(A*n(2,3,2)+A*n(3,2,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3)-...
            (H3-p1/p11)*(p13*p4/p11^2-(A*n(2,3,3)+A*n(3,2,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta^2;
 
        %Totally same as dxdz

        % dzdx = (-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-...
        %     (H1-p3/p11)*(p13*p6/p11^2-(A*n(1,3,1)+A*n(3,1,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3)-...
        %     (H2-p2/p11)*(p13*p5/p11^2-(A*n(1,3,2)+A*n(3,1,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3)-...
        %     (H3-p1/p11)*(p13*p4/p11^2-(A*n(1,3,3)+A*n(3,1,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta^2;
 

        %Totally same as dydz
        % dzdy = (-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-...
        %     (H1-p3/p11)*(p13*p6/p11^2-(A*n(2,3,1)+A*n(3,2,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3)-...
        %     (H2-p2/p11)*(p13*p5/p11^2-(A*n(2,3,2)+A*n(3,2,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3)-...
        %     (H3-p1/p11)*(p13*p4/p11^2-(A*n(2,3,3)+A*n(3,2,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta^2;
 

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
        dzdz = (-(p7/p8-p3*p9/p8^2)^2-(p6/p8-p2*p9/p8^2)^2-(p5/p8-p1*p9/p8^2)^2-...
            (H1-p3/p8)*(2*p9*p7/p8^2+p3*p4/p8^2-2*A*n(3,3,1)/p8-2*p3*p9^2/p8^3)-...
            (H2-p2/p8)*(2*p9*p6/p8^2+p2*p4/p8^2-2*A*n(3,3,2)/p8-2*p2*p9^2/p8^3)-...
            (H3-p1/p8)*(2*p9*p5/p8^2+p1*p4/p8^2-2*A*n(3,3,3)/p8-2*p1*p9^2/p8^3))/delta^2;


        FisherImatrix = [dxdx dxdy dxdz; dxdy dydy dydz; dxdz dydz dzdz];


        I = -FisherImatrix;
        B = inv(I);
        CRLB(i) = trace(B);
        output = [i/step_len+1 j/step_len+1]
  
end

figure
semilogy(SNR,CRLB)
hold on
test_point = [3 0 0.5];
for i=1:length(SNR)

        Var_Noise = Theory_Power(test_point, P_led, P_led-test_point, N_led, m, M, Ar)/10^(SNR(i)/10);
        delta = sqrt(Var_Noise);
    
        x =  test_point(1) - P_led(1);
        y =  test_point(2) - P_led(2);
        z =  test_point(3) - P_led(3);
        N_pd = Facing_Vector(test_point, P_led);

        n(:,:,1) = -transpose(N_led)*N_pd(1,:);
        n(:,:,2) = -transpose(N_led)*N_pd(2,:);
        n(:,:,3) = -transpose(N_led)*N_pd(3,:);
 
        H1 = Theory_Power(test_point, P_led, N_pd(1,:), N_led, m, M, Ar);
        H2 = Theory_Power(test_point, P_led, N_pd(2,:), N_led, m, M, Ar);
        H3 = Theory_Power(test_point, P_led, N_pd(3,:), N_led, m, M, Ar);

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
        dxdx = (-(p7/p8-p3*p9/p8^2)^2-(p6/p8-p2*p9/p8^2)^2-(p5/p8-p1*p9/p8^2)^2-...
            (H1-p3/p8)*(2*p9*p7/p8^2+p3*p4/p8^2-2*A*n(1,1,1)/p8-2*p3*p9^2/p8^3)-...
            (H2-p2/p8)*(2*p9*p6/p8^2+p2*p4/p8^2-2*A*n(1,1,2)/p8-2*p2*p9^2/p8^3)-...
            (H3-p1/p8)*(2*p9*p5/p8^2+p1*p4/p8^2-2*A*n(1,1,3)/p8-2*p1*p9^2/p8^3))/delta^2;
 

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
        dxdy = (-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-...
            (H1-p3/p11)*(p13*p6/p11^2-(A*n(1,2,1)+A*n(2,1,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3)-...
            (H2-p2/p11)*(p13*p5/p11^2-(A*n(1,2,2)+A*n(2,1,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3)-...
            (H3-p1/p11)*(p13*p4/p11^2-(A*n(1,2,3)+A*n(2,1,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta^2;
 

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
        dxdz = (-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-...
            (H1-p3/p11)*(p13*p6/p11^2-(A*n(1,3,1)+A*n(3,1,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3)-...
            (H2-p2/p11)*(p13*p5/p11^2-(A*n(1,3,2)+A*n(3,1,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3)-...
            (H3-p1/p11)*(p13*p4/p11^2-(A*n(1,3,3)+A*n(3,1,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta^2;
 
        %Totally same as dxdy

        % dydx = (-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-...
        %     (H1-p3/p11)*(p13*p6/p11^2-(A*n(1,2,1)+A*n(2,1,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3)-...
        %     (H2-p2/p11)*(p13*p5/p11^2-(A*n(1,2,2)+A*n(2,1,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3)-...
        %     (H3-p1/p11)*(p13*p4/p11^2-(A*n(1,2,3)+A*n(2,1,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta^2;
 

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
        dydy = (-(p7/p8-p3*p9/p8^2)^2-(p6/p8-p2*p9/p8^2)^2-(p5/p8-p1*p9/p8^2)^2-...
            (H1-p3/p8)*(2*p9*p7/p8^2+p3*p4/p8^2-2*A*n(2,2,1)/p8-2*p3*p9^2/p8^3)-...
            (H2-p2/p8)*(2*p9*p6/p8^2+p2*p4/p8^2-2*A*n(2,2,2)/p8-2*p2*p9^2/p8^3)-...
            (H3-p1/p8)*(2*p9*p5/p8^2+p1*p4/p8^2-2*A*n(2,2,3)/p8-2*p1*p9^2/p8^3))/delta^2;
 

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
        dydz = (-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-...
            (H1-p3/p11)*(p13*p6/p11^2-(A*n(2,3,1)+A*n(3,2,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3)-...
            (H2-p2/p11)*(p13*p5/p11^2-(A*n(2,3,2)+A*n(3,2,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3)-...
            (H3-p1/p11)*(p13*p4/p11^2-(A*n(2,3,3)+A*n(3,2,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta^2;
 
        %Totally same as dxdz

        % dzdx = (-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-...
        %     (H1-p3/p11)*(p13*p6/p11^2-(A*n(1,3,1)+A*n(3,1,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3)-...
        %     (H2-p2/p11)*(p13*p5/p11^2-(A*n(1,3,2)+A*n(3,1,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3)-...
        %     (H3-p1/p11)*(p13*p4/p11^2-(A*n(1,3,3)+A*n(3,1,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta^2;
 

        %Totally same as dydz
        % dzdy = (-(p9/p11-p3*p13/p11^2)*(p6/p11-p3*p12/p11^2)-(p8/p11-p2*p13/p11^2)*(p5/p11-p2*p12/p11^2)-(p7/p11-p1*p13/p11^2)*(p4/p11-p1*p12/p11^2)-...
        %     (H1-p3/p11)*(p13*p6/p11^2-(A*n(2,3,1)+A*n(3,2,1))/p11+p12*p9/p11^2+p10*p3/p11^2-2*p3*p13*p12/p11^3)-...
        %     (H2-p2/p11)*(p13*p5/p11^2-(A*n(2,3,2)+A*n(3,2,2))/p11+p12*p8/p11^2+p10*p2/p11^2-2*p2*p13*p12/p11^3)-...
        %     (H3-p1/p11)*(p13*p4/p11^2-(A*n(2,3,3)+A*n(3,2,3))/p11+p12*p7/p11^2+p10*p1/p11^2-2*p1*p13*p12/p11^3))/delta^2;
 

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
        dzdz = (-(p7/p8-p3*p9/p8^2)^2-(p6/p8-p2*p9/p8^2)^2-(p5/p8-p1*p9/p8^2)^2-...
            (H1-p3/p8)*(2*p9*p7/p8^2+p3*p4/p8^2-2*A*n(3,3,1)/p8-2*p3*p9^2/p8^3)-...
            (H2-p2/p8)*(2*p9*p6/p8^2+p2*p4/p8^2-2*A*n(3,3,2)/p8-2*p2*p9^2/p8^3)-...
            (H3-p1/p8)*(2*p9*p5/p8^2+p1*p4/p8^2-2*A*n(3,3,3)/p8-2*p1*p9^2/p8^3))/delta^2;


        FisherImatrix = [dxdx dxdy dxdz; dxdy dydy dydz; dxdz dydz dzdz];


        I = -FisherImatrix;
        B = inv(I);
        CRLB(i) = trace(B);
        output = [i/step_len+1 j/step_len+1]
  
end
semilogy(SNR,CRLB)
legend('In the center','In the edge')
xlabel('SNR(dB)')
ylabel('MSE')
title('The CRLB In the center of area')
grid on





