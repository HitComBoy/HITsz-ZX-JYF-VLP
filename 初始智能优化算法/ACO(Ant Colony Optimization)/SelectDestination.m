function destination = SelectDestination(nowLocation,  notReachedCities, pathes)
%������ nowLocation=��ǰ���ڳ��е�����
%ǰ�� notReachedCities=δ�����ĳ��е������б� �е�һ������
%ѡ�񷽷�: ����pathes.pheromone����Ϣ��Ũ�� �����̶�ѡ��

citiesCnt = length(notReachedCities);
pheromone = zeros(citiesCnt, 1);%nowLocation��notReachedCities֮�����Ϣ��
for i = 1: citiesCnt
    pheromone(i) = pathes.pheromone(nowLocation, notReachedCities(i));%��·���ϵ���Ϣ��
end

%������Ϣ�� ���̶�ѡ��Ŀ�ĵ�
index = Roulette(pheromone);%�������̶���ѡ�������(����pheromone)
destination = notReachedCities(index);%��ö�Ӧ�ĳ��е�����

end

