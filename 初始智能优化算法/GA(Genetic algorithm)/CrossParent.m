function childGene = CrossParent(parent, individual)
%parent=����ĸ���ı��
%individualÿ��=[������, �ø������ڵ�����x] �� ����=������

%ĸ���ı��
parent1Index = parent(1);
parent2Index = parent(2);
%ĸ��������x
parent1X = individual(parent1Index, 2);
parent2X = individual(parent2Index, 2);
%ĸ���Ļ���
parent1Gene = num2gene( parent1X );%������
parent2Gene = num2gene( parent2X );%������

%����=2��ĸ����1~cutλ�Ļ�����н���
cut = floor( rand(1,1) * 12 ) + 1;%����ȡ��=1~12������
for i=1: 1: cut
    tmp = parent1Gene(i);
    parent1Gene(i) = parent2Gene(i);
    parent2Gene(i) = tmp;
end

%�Ӵ�=������ĸ��
childGene = [parent1Gene; parent2Gene];%������

end

