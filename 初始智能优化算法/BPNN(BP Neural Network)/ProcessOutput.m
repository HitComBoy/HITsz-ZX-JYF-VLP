function data = ProcessOutput(data)
%����data����Ԫ���������
%���о���������ֵ����Ԫ ��1(�������������)
%������Ԫ����� ��-1
%data = [���۽��, ��������, ��������������]

column = size(data, 2);

%����data���� �ҵ����ֵ���±�
index = 1;
for columnIndex = 2: column
    if data(columnIndex-1) < data(columnIndex)
        index = columnIndex;
    end
end

%���ֵ��Ӧ����Ԫ��� ��1
%�������Ԫ��� ��-1
for columnIndex = 1: column
    if columnIndex == index
        data(columnIndex) = 1;
    else
        data(columnIndex) = -1;
    end
end

end

