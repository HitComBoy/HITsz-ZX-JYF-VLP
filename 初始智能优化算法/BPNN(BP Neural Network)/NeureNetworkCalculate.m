function network = NeureNetworkCalculate(network, data)
%���һ��������ļ���
%����data������X������ ����������ĸ������

%ÿ����Ԫ�ĸ���
inputNeureNum  = size(network.inputNeure, 2);
hiddenNeureNum = size(network.hiddenNeure, 2);
outputNeureNum = size(network.outputNeure, 2);

%1) �������������
layerInput = data;%�������������� = data��������

for neureIndex = 1: inputNeureNum%ÿ����Ԫ
    singleNeure = network.inputNeure(neureIndex);%������Ԫ
    %[fx, sum] = NeureCalculate(singleNeure, layerInput);%��Ԫ����+����
    [fx, sum] = NeureCalculate(singleNeure, layerInput);%��Ԫ����+����
    
    network.inputMeure(neureIndex).input = layerInput;
    network.inputNeure(neureIndex).output = fx;
    network.inputNeure(neureIndex).netSum = sum;
end

%2) ��������������
layerInput = zeros(1, inputNeureNum);%��������������� = ���������
for neureIndex = 1: inputNeureNum%������ÿ����Ԫ
    layerInput(neureIndex) = network.inputNeure(neureIndex).output;
end

for neureIndex = 1: hiddenNeureNum%ÿ����Ԫ
    singleNeure = network.hiddenNeure(neureIndex);%������Ԫ
    [fx, sum] = NeureCalculate(singleNeure, layerInput);%��Ԫ����+����
    
    network.hiddenNeure(neureIndex).input = layerInput;
    network.hiddenNeure(neureIndex).output = fx;
    network.hiddenNeure(neureIndex).netSum = sum;
end

%3) �������������
layerInput = zeros(1, hiddenNeureNum);%�������������� = ����������
for neureIndex = 1: hiddenNeureNum%������ÿ����Ԫ
    layerInput(neureIndex) = network.hiddenNeure(neureIndex).output;
end

for neureIndex = 1: outputNeureNum%ÿ����Ԫ
    singleNeure = network.outputNeure(neureIndex);%������Ԫ
    [fx, sum] = NeureCalculate(singleNeure, layerInput);%��Ԫ����+����
    
    network.outputNeure(neureIndex).input = layerInput;
    network.outputNeure(neureIndex).output = fx;
    network.outputNeure(neureIndex).netSum = sum;
end
        
end

