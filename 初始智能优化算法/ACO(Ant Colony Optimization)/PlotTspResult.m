function passByCities = PlotTspResult(cities, passByCities, figNum)
%����TSP�Ľ��

%cities=ÿ�����е�XY����
%passByCities=TSP���򾭹��ĳ��е�XY����
%figNum=ͼ��

figure(figNum);
PlotCities(cities);

line(passByCities(:,1), passByCities(:,2));

end

