function parents = GetTwoParents( fitness )
%������Ӧ��ֵ ���̶ķ� ��ȡ����ĸ��
%(��Ӧ�ȴ�� �ϴ�ĸ��ʱ�ѡΪĸ��=�����Ŵ��е���������)

%����ĸ����������� ���һ��
parents = zeros(1, 2);

num = size(fitness, 1);
sum = 0;%��Ӧֵ�ܺ�
for i = 1: 1: num
    sum = sum + fitness(i,2);
end

%���̶� ȷ������ĸ��
randomValue = rand(1, 2);
possibility = randomValue .* sum;
for parentNum = 1: 1: 2%����ĸ��
    tmpSum = 0;
    for i = 1: 1: num
        tmpSum = tmpSum + fitness(i,2);
        if tmpSum >= possibility(parentNum)
            parents(parentNum) = fitness(i,1);%ĸ�����
            break;
        end
    end 
end

end
