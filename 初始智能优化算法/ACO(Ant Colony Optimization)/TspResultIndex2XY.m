function tspResultXY = TspResultIndex2XY(tspResultIndex, cities)
%�����е����� ת��Ϊ ���е�XY����

%tspResultIndex=��˳�򾭹��ĳ��е�����
%cities=ÿ�����е�XY����

citiesCnt = length(tspResultIndex);%citiesNum+1(�������Ļص����)
tspResultXY = zeros(citiesCnt, 2);

for i = 1: citiesCnt%ÿ������
    destCityIndex = tspResultIndex(i);
    tspResultXY(i, :) = cities(destCityIndex, :);%��i�ξ����ĳ��� ������
end
    
end

