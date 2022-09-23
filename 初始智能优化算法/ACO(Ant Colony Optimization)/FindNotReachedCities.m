function notReachedCities = FindNotReachedCities(ants, antsIndex, pathes, searchRange)
%���ص�antsIndexֻ���� δ������ĳ���

%ants=��Ⱥ
%antsIndex=��Ⱥ�еĵڼ�ֻ����
%pathes=����·��
%searchRange=ÿֻ���� ���ܹ۲쵽/�����ķ�Χ

notReachedCities=[];

citiesNum = size(ants.cities, 2);
nowLocation = ants.location(antsIndex);


cnt = 0;
for i = 1: citiesNum%�����б�
    distance = pathes.length(nowLocation, i);%�����м�ľ���
    if ants.cities(antsIndex, i) == 0 && distance < searchRange%�ó���δ����� �� �ó������������ܸ�֪�ķ�Χ��
        cnt = cnt+1;
        notReachedCities(cnt) = i;
    end
end

%����ڸ÷�Χ��û��δ������ĳ��� ������Χ�ٴ�find
if length(notReachedCities) == 0
   notReachedCities = FindNotReachedCities(ants, antsIndex, pathes, searchRange*1.5);
   return;
end

end

