function [fx, sum] = NeureCalculate(neure, input)
%���� ������Ԫneure�����Ȩ�� �� ����Ԫ����������input
%���� ������Ԫ�ļ������output

%���ά��
lenInput = length(input);
lenNeure = length(neure.input);
if lenInput ~= lenNeure
    disp(['error dim for input/neure.input=', num2str(lenInput), '/', num2str(lenNeure)]);
    return;
end

%��;�����
sum = 0;
for i = 1: lenInput%����ÿ������
    sum = sum + input(i)*neure.weight(i);%����i*Ȩ��i
end
% if length(neure.weight) == lenInput+1%ƫ����Ԫx0=-1
%     sum = sum - neure.weight(lenInput+1);
% end

%�����������fx
fx = ActiveFunc(sum);

end

