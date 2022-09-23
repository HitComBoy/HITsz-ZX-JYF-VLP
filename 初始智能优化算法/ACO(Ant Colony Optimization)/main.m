%��Ⱥ�㷨�������������TSP
clc; clear all; close;

%һЩ����ѭ���Ĵ���
optimalizeTimes = 2;%��Ⱥ�㷨���ÿ�ʼ�Ĵ���(������¿�ʼ��Ⱥ�㷨 Ѱ�����·���Ľ�)
cycle = 100;%���г��о�������ѭ������

%��ʼ������ = ����λ��+·�����Ⱥ���Ϣ��Ũ��
cities = InitCities();%ÿ��=һ�����е�����
citiesNum = size(cities, 1);
pathes = InitPath(cities);%�������м�ĳ���length����Ϣ��pheromone
PlotCities(cities);

%��ʼ����Ⱥ
%location(i)=��iֻ���ϵĵ�ǰλ��
%cities(i,:)=��iֻ���ϵ�����ĳ���(ֵ=�ó��е����˳��)
%destination(i)=��iֻ����ѡ���Ŀ�ĵ�
%length(i)=��iֻ���Ͼ����ĳ��е�·������
ants = InitAnts(citiesNum);%ÿ��=һֻ���� ��¼�����ϵĵ�ǰλ��location�͵�����ĳ���cities
antsNum = size(ants.location, 1);
secretedPheromone = 2;%ÿֻ���� ÿ���ڳ��м��˶� ������������Ϣ�ص���
searchRange = 10;%ÿֻ���� ���ܹ۲쵽/�����ķ�Χ

%��¼TSP����ѽ��
%��optimalizeTimesѭ���б��Ż�
optimalTspResultIndex = [];
optimalTspResultLength = 100000;



for optimalIndex = 1: optimalizeTimes
    %���¿�ʼTSPʵ�� ���·���ϵ���Ϣ��
    %������¿�ʼ��Ⱥ�㷨 Ѱ�����·���Ľ�
    %Step1: ��ʼ������
    pathes = InitPath(cities);
    
    for cycleIndex = 1: cycle%ÿ�ֱ������г���
        %���г��б���һ�κ� ������Ⱥ״̬
        %Step2: ��ʼ����Ⱥ
        ants = InitAnts(citiesNum);%RestoreAnts(ants);

        %loop: �������г���
        %loop��2��ʼ(1=��һ���˶� �趨������)
        %��citiesNum+1����(citiesNum+1=���һ���˶� �ص�������)
        loopTimes = citiesNum + 1;
        for loopIndex = 2: loopTimes%ÿ��loop����һ������
            %Step3: ÿֻ���� ������Ϣ��ѡ��Ŀ�ĵس���(��δ������ĳ�����ѡ��)
            if loopIndex ~= loopTimes
                 %�ӵ�ǰλ��nowLocation---��û�е�����ĳ�����notReachedCities
                 %���ݵ�ͼ�ϵ���Ϣ�طֲ�---ѡ��һ��Ŀ�ĵ�
                for antsIndex = 1: antsNum%ÿֻ����
                    nowLocation = ants.location(antsIndex);%��ǰ���ڳ��е�����
                    notReachedCities = FindNotReachedCities(ants, antsIndex, pathes, searchRange);%δ�����ĳ��е������б�
                    ants.destination(antsIndex) = SelectDestination(nowLocation, notReachedCities, pathes);
                end
            else%���һ�� ���г��о������ �ص�������
                for antsIndex = 1: antsNum%ÿֻ����
                    ants.destination(antsIndex) = FindStartCity(ants, antsIndex);
                end
            end

            %Step4: ÿֻ���� ǰ��Ŀ�ĵ�, ͬʱ����pathes�ϵ���Ϣ�طֲ�
            for antsIndex = 1: antsNum%ÿֻ����
                nowLocation = ants.location(antsIndex);%��ǰ���ڵĳ���
                destination = ants.destination(antsIndex);%Ŀ�ĵس���

                %�����ƶ�
                if loopIndex ~= loopTimes%�������һ��(�ص������� ����Ҫ��¼)
                    ants.cities(antsIndex, destination) = loopIndex;%��¼�ǵڼ��������ĳ���=rem(loopIndex, loopTimes)
                end
                ants.location(antsIndex) = destination;
                movingLength = pathes.length(nowLocation, destination);
                ants.length(antsIndex) = ants.length(antsIndex) + movingLength;

                %���µ�ͼ�ϵ���Ϣ�طֲ�(ÿ��������һ������ ��������Ϣ��Ũ��=���ڵ���Ϣ��/���м�ľ���)
                nowPheromone = pathes.pheromone(nowLocation, destination);
                incPheromone = secretedPheromone / movingLength;%�������˶������ӵ���Ϣ��Ũ��
                pathes.pheromone(nowLocation, destination) = nowPheromone + incPheromone;
            end

            %Step5: pathes�ϵ���Ϣ�ػӷ�
            for i = 1: citiesNum
                for j = 1: citiesNum
                    nowPheromone = pathes.pheromone(i, j);
                    pathes.pheromone(i,j) = nowPheromone*0.95;
                end
            end
        end%��Ⱥ�������г���
    end%��Ⱥ�������г���cycle��

    %Step7: ��Ⱥ�������г���cycle�κ� ��¼���ֵ�TSP���
    tspResultIndex = FindTspResult(pathes);%TSP˳�򾭹��ĳ��е�����
    %����TSP��·����
    totalLength = 0;
    for i = 1: length(tspResultIndex)-1
        movingLength = pathes.length(tspResultIndex(i), tspResultIndex(i+1));
        totalLength = totalLength + movingLength;
    end
    disp(['TSP total length=', num2str(totalLength)]);
    
    %Step8: ��¼��ѵ�TSP���(�����Ⱥ�㷨 Ѱ�����·���Ľ��Ž�)
    if totalLength < optimalTspResultLength
        optimalTspResultLength = totalLength;
        optimalTspResultIndex = tspResultIndex;
        disp(['optimalTspResultLength is updated as ', num2str(optimalTspResultLength)]);
        
        passByCities = TspResultIndex2XY(optimalTspResultIndex, cities);
        tspResult = PlotTspResult(cities, passByCities, optimalIndex);
    end
end

% passByCities = TspResultIndex2XY(optimalTspResultIndex, cities);
% tspResult = PlotTspResult(cities, passByCities, 2);
