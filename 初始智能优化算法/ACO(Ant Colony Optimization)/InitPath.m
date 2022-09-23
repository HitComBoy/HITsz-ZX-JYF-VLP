function path = InitPath( cities )
%��ʼ�����м��·��: ���м��·���ĳ���+��Ϣ��
%���м�����֮����ͨ·

%citiesÿ��=һ�����е�����
initPheromone = 1;

cityNum = size(cities, 1);

%��������֮���·������
path.length = zeros(cityNum, cityNum);
for i = 1: cityNum%ÿ������ �� �������м�ľ���
    for j = 1: cityNum
        path.length(i,j) = GetLength(cities(i,:), cities(j,:));
    end
end

%��������֮�����Ϣ��
path.pheromone = zeros(cityNum, cityNum);
for i = 1: cityNum%ÿ������ �� �������м����Ϣ��
    for j = 1: cityNum
        path.pheromone(i,j) = initPheromone;
    end
end

end

