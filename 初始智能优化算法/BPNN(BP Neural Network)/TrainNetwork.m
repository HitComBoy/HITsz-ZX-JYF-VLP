function network = TrainNetwork(network, data, xDim, epochs, learningRate)
%����dataѵ��������network
%X��ά��xDim
%epochsѧϰѭ������
%learningRateѧϰ�ٶȳ���

%see: ������֮�ݶ��½��뷴�򴫲����£�
% https://zhuanlan.zhihu.com/p/25609953

dataNum = size(data, 1);%�������ݵ�����

for epochsIndex = 1: epochs%ѵ������
    for dataIndex = 1: dataNum%ÿ��ѵ������
        %1) ������ļ���
        inputData = data(dataIndex, 1:xDim);%��������������� = ��һ����ʵ�����ݵ�����γ������
        network = NeureNetworkCalculate(network, inputData);
        
        %2) ѧϰѵ��
        outputData = data(dataIndex, xDim+1:end);%
        network = BackPropagation(network, outputData, learningRate, epochsIndex);
    end
end

end

