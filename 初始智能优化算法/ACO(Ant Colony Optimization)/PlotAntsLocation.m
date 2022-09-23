function PlotAntsLocation(ants, cities, figNum)
%����ants.location�м�¼ �����������ϵ�λ��

antsNum = length(ants.location);

 figure(figNum);
 PlotCities(cities);
 
 antsLocation = zeros(antsNum, 2);
 for antsIndex = 1: antsNum%ÿֻ����
     citiesIndex = ants.location(antsIndex);
     antsLocation(antsIndex, :) = cities(citiesIndex, :);
 end
 
 plot(antsLocation(:,1), antsLocation(:,2), 'bo');
 
end

