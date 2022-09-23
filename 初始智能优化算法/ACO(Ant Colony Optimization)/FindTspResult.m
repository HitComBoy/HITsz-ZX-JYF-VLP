function tspResult = FindTspResult(pathes)
%����pathes.pheromone
%����ѡ�������Ϣ��Ũ�ȵ�·��
%���TSP����Ľ�(��˳�򾭹��ĳ��е�����)

citiesNum = size(pathes.pheromone, 1);
tspResult = zeros(citiesNum+1, 1);%+1�ص����

%������
initCityIndex = 1;%fix(rand(1)*citiesNum);
startCityIndex = initCityIndex;
tspResult(1) = initCityIndex;

for citiesIndex = 2: citiesNum%ѡ��ڼ��������ĳ���
    %ѡ��û�����ĳ���possibleCities
    leftCitiesNum = citiesNum-citiesIndex+1;%û�����ĳ�������
    possibleCities = zeros(leftCitiesNum,1);
    cnt = 0;
    for i = 1 : citiesNum%���г���
        if ismember(i, tspResult) == 0%i����û����
            cnt = cnt+1;
            possibleCities(cnt) = i;
        end
    end
    
    %��û�����ĳ��е��� ѡ��һ������ ����Ϣ��Ũ�����
    maxPheromone = 0;
    for i = 1: leftCitiesNum
        possibleCityIndex = possibleCities(i);
        possibleCityPheromone = pathes.pheromone(startCityIndex, possibleCityIndex);
        
        if maxPheromone < possibleCityPheromone;
            maxPheromone = possibleCityPheromone;
            destCityIndex = possibleCityIndex;
        end
    end
    
    %ǰ��Ŀ�ĳ���
    tspResult(citiesIndex) = destCityIndex;
    startCityIndex = destCityIndex;
end

%�ص�������
tspResult(citiesNum+1) = initCityIndex;

end

