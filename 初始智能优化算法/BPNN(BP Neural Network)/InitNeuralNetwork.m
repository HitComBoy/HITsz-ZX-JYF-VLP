function network = InitNeuralNetwork(inputNum, hiddenNum, outputNum, xDim, epochs)
%inputNum  ��������Ԫ����
%hiddenNum ���������Ԫ����
%outputNum ��������Ԫ����
%xDim      ���ݵ�Xά��
%epochs    ѧϰѭ������

%ÿ����Ԫ��Ȩ�ص�ά�� = ǰһ��������ά��
%Ȩ�ط�Χ-1~1

%�����
for i = 1: inputNum
    singleNeure.weight = 2*rand(xDim,1)-1;%xDim
    singleNeure.input = zeros(xDim, 1);
    singleNeure.output = 0;
    singleNeure.netSum = 0;
    network.inputNeure(i) = singleNeure;
end

%������
for i = 1: hiddenNum
    singleNeure.weight = 2*rand(inputNum,1)-1;%inputNum
    singleNeure.input = zeros(inputNum, 1);
    network.hiddenNeure(i) = singleNeure;
end

%�����
for i = 1: outputNum
    singleNeure.weight = 2*rand(hiddenNum,1)-1;%hiddenNum
    singleNeure.input = zeros(hiddenNum, 1);
    network.outputNeure(i) = singleNeure;
end

%ÿ��ѧϰѭ�� ��������Ԫ�����
network.error = zeros(epochs, outputNum);

end

