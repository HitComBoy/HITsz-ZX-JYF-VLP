function ants = InitAnts(citiesNum)
%��ʼ����Ⱥ: ÿֻ���ϵ�λ��+������ĳ���+��ǰ��Ŀ�ĵ�+��������е�·���ĳ���

antsNum = 30;
ants.location = zeros(antsNum, 1);%ÿֻ���ϵĵ�ǰλ��
ants.cities = zeros(antsNum, citiesNum);%ÿ��=һֻ���� ��¼�����ϵ�����ĳ���
ants.destination = zeros(antsNum, 1);%ÿֻ����ѡ���Ŀ�ĵ�
ants.length = zeros(antsNum, 1);%ÿֻ�����Ѿ����ĳ��е�·������

for i = 1: antsNum%ÿֻ���� ����趨��ʼλ��
    startCity = round( rand(1)*(citiesNum-1) ) + 1;%(0~num-1)+1=�±��1��ʼ
    
    ants.location(i) = startCity;
    ants.cities(i, startCity) = 1;%�����ĵ�һ������
    ants.destination(i) = startCity;
    ants.length(i) = 0;
end

end

