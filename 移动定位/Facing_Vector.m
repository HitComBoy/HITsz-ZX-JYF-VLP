%Find the Facing Vector which related to the position（with the orientation num K）
function Fvector = Facing_Vector(P_pd , P_led , K)
    
    Vector_reference = P_led - P_pd ; %Vector from PD to LED, the final vectors we need is three vectors around it.
    Vector_reference= Vector_reference/norm(Vector_reference,2);
    Theta = asin(Vector_reference(3)/1); %Angle between V_R and ground
    Mid_Vector = [Vector_reference(1) Vector_reference(2) 0];
    if Vector_reference(1)^2+Vector_reference(2)^2 ~= 0
        Mid_Vector = Mid_Vector/norm(Mid_Vector,2)*sqrt(tan(Theta)^2/(3*(1+tan(Theta)^2)));       
        Mid_Vector(3) = -sqrt(1/(3*(1+tan(Theta)^2)));
        Fvector(1,:) = Vector_reference + Mid_Vector;
        Fvector(1,:)= Fvector(1,:)/norm(Fvector(1,:),2);
        for i=2:K
            Fvector(i,:) = cos((i-1)*2*pi/K)*Fvector(1,:) + (1-cos((i-1)*2*pi/K))*dot(Vector_reference,Fvector(1,:))*Vector_reference + sin((i-1)*2*pi/K)*(cross(Vector_reference,Fvector(1,:)));
        end
    else
        Mid_Vector = [1/sqrt(6) 1/sqrt(6) 0];
        Fvector(1,:) = Vector_reference + Mid_Vector;
        Fvector(1,:)= Fvector(1,:)/norm(Fvector(1,:),2);
        for i=2:K
            Fvector(i,:) = cos((i-1)*2*pi/K)*Fvector(1,:) + (1-cos((i-1)*2*pi/K))*dot(Vector_reference,Fvector(1,:))*Vector_reference + sin((i-1)*2*pi/K)*(cross(Vector_reference,Fvector(1,:)));
        end
    end
end


