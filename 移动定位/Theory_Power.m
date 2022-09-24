function Power = Theory_Power(P_pd, P_led, N_pd, N_led, m, M, Ar)
% we assume the transform power equal to 1, in this case Channel impulse
% reponse is equal to recieve power in numerical
    if size(N_pd,1) == 1
        D_led = P_pd  - P_led;
        D_pd  = P_led - P_pd;
        Theta = dot(D_led,N_led)/(norm(D_led,2)*norm(N_led,2)) ;%Irradiance Angle(Cos value)
        Psi   = dot(D_pd,N_pd)/(norm(D_pd,2)*norm(N_pd,2)) ;%Incidence Angle
        if Psi < 0
            Psi =0;  %Incidence Angle maybe larger than 90, in this case we handle it as 90
        end
        Power = (m+1)*Ar*(Theta^m*Psi^M)/(2*pi*(norm(D_led,2)^2));
    else
        D_led = P_pd  - P_led;
        D_pd  = P_led - P_pd;
        muti_Power = zeros(1,size(N_pd,1));
        for i=1:size(N_pd,1)
            Theta = dot(D_led,N_led)/(norm(D_led,2)*norm(N_led,2)) ;%Irradiance Angle(Cos value)
            Psi   = dot(D_pd,N_pd(i,:))/(norm(D_pd,2)*norm(N_pd(i,:),2)) ;%Incidence Angle
            if Psi < 0
                Psi =0;  %Incidence Angle maybe larger than 90, in this case we handle it as 90
            end
            muti_Power(i) = (m+1)*Ar*(Theta^m*Psi^M)/(2*pi*(norm(D_led,2)^2));
        end
        Power = muti_Power;
    end
end

