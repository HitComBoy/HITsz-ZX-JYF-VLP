function fitness = GetFitness( individual )
%����������Ӧ��

[r, c] = size(individual);
fitness = zeros(r, c);
for i = 1: 1: r
    fitness(i, 1) = i;%������
    
    x = individual(i,2);%�ø�������Ӧ������x
    fitness(i, 2) = x+10*sin(5*x)+7*cos(4*x);%��Ӧ��=����ֵ
end

end

