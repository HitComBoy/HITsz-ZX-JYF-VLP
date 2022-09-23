k=20000;
miderr=zeros(1,k);
parfor i=1:k
    P_pd = [1.5 1.5 0.5];
    particle = [3*rand(1) 3*rand(1) 3*rand(1)];
    miderr(i) = norm((P_pd-particle),2)^2;        
end
err=mean(miderr);