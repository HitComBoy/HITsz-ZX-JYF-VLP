function result = TestNetwork(network, data, xDim)
%����������network����data�����result
%X��ά��xDim

%result = [���۽��, ��������, ��������������]

dataNum = size(data, 1);%�������ݵ�����
yDim = size(data, 2) - xDim;
result = zeros(dataNum, 3*yDim);

for dataIndex = 1: dataNum%ÿ��ѵ������
    %1) ������ļ���
    inputData = data(dataIndex, 1:xDim);%��������������� = ��һ����ʵ�����ݵ�����γ������
    network = NeureNetworkCalculate(network, inputData);
    %2) ��¼���
    result(dataIndex, 1:yDim) = data(dataIndex, xDim+1:end);%���۽��
    for yIndex = 1: yDim
        result(dataIndex, yDim+yIndex) = network.outputNeure(yIndex).output;%��������
    end
    %3) ����������Ľ��
    result(dataIndex, 2*yDim+1:end) = ProcessOutput(result(dataIndex, yDim+1:2*yDim));
end

end

