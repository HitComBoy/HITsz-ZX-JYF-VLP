function selectedIndex = Roulette(values)
%����values���б��е�ֵ(��ȷ�� ֵȫ��Ϊ��)
%���̶�ѡ��һ��Ŀ��
%���ظ�Ŀ�����б��е�����

len = length(values);

%������ֵ�ĺ�
sum = 0;
for i = 1 : len
    sum = sum + values(i);
end

%��0~sum֮��ȡһ�����ֵ
randomValue = rand(1)*sum;

%�ҵ��ۼƺ�>=randomValue�������±�
sum = 0;
for i = 1 : len
    sum = sum + values(i);
    if sum >= randomValue
        selectedIndex = i;
        return;
    end
end

end

