function num = gene2num( gene )
%12λ�Ļ������(6λ��������+6λС������) ת��Ϊ һ����

num = 0;
for i = 1: 1: 12
	num = num + gene(i)*10^(i-7);
end

if num<15 || num >25%�����ĺ����xԽ�� �����ֵ
    num = 15 + 10*rand(1, 1);
end


end

