function startCityIndex = FindStartCity(ants, antsIndex)
%�ҵ�ants�е�antsIndexֻ���� �˶��ĵ�һ��������

citiesNum = size(ants.cities, 2);
for i = 1: citiesNum
    if ants.cities(antsIndex, i) == 1%������
        startCityIndex = i;
        return;
    end
end

end

