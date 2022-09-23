function [result, maxVector, minVector] = NormalizeData(data)
%������data���й�һ��
%data=[x1,x2,x3,x4, y]
%dataÿ��=һ����������
%dataÿ��=һ������

[row, column] = size(data);

%result = -1*ones(row, column+2);%y��Ϊ[y1 y2 y3] ��Ӧ3ά���
result = zeros(row, column+2);%y��Ϊ[y1 y2 y3] ��Ӧ3ά���
for rowIndex = 1: row
    switch data(rowIndex, column)
        case 1%y1=1
            result(rowIndex, column) = 1;
        case 2%y2=1
            result(rowIndex, column+1) = 1;
        case 3%y3=1
            result(rowIndex, column+2) = 1;
    end
end

%ÿ������X�������Сֵ
minVector = min(data(:, 1:column-1), [], 1);%һ��������=ÿ�е���Сֵ
maxVector = max(data(:, 1:column-1), [], 1);%һ��������=ÿ�е����ֵ
for columnIndex = 1: column-1%ÿ��=ÿ������X ���й�һ��
    %���е������Сֵ
    minNum = minVector(columnIndex);
    maxNum = maxVector(columnIndex);
    
    for rowIndex = 1: row%ÿ�� ��������ÿ������
        x = data(rowIndex, columnIndex);
        result(rowIndex, columnIndex) = 2 * (x-minNum) / (maxNum-minNum) - 1;%����ת�� ��һ����[-1,1]
    end
end

end

