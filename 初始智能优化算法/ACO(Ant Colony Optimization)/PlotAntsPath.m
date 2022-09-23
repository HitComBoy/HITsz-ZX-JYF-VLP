function PlotAntsPath(ants, cities, figNum)
%����ants.cities�м�¼��˳�� �������ӳ���

antsNum = length(ants.location);
citiesNum = size(cities, 1);

 figure(figNum);
 hold on;
 PlotCities(cities);
 
 passByCities = zeros(citiesNum, 2);
 for antsIndex = 1: 1%antsNum%ÿֻ����
     for citiesIndex = 1: citiesNum%ÿ������
         order = ants.cities(antsIndex, citiesIndex);%����citiesIndex���е�ʵ�ʴ���
         passByCities(order, :) = cities(citiesIndex,:);%��order�ξ����ĳ��� ������
     end
     plot(passByCities(:,1), passByCities(:,2), 'b');
 end
 
end

