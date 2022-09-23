%BP������(������ȶ� �����м��������ֽϺõĽ��)
%���Ժ�main_matlab_toolbox.m�Ľ�����бȽ�
%����ж�iris����Ʒ��
%see: http://blog.csdn.net/luoshengkim/article/details/41923723

clc;clear;close all;

%�������ѵ������
epochs= 500;
%ѧϰ�ٶ�
learningRate = 0.0001;

xDim = 4;%[x1, x2, x3, x4] 4������
yDim = 3;%[y1, y2, y3] 3��Ʒ��

%1) ��ȡѵ��������
trainData = ReadData('TrainData.txt');
%��һ������ʵ������
[normalizedTrainData, maxTrainVector, minTrainVactor]= NormalizeData(trainData);

%2) ��ʼ��������=4����-5����-3���(��Y��ά��)-4άX
network = InitNeuralNetwork(7,14,3, xDim, epochs);

%3) ѵ��������
network = TrainNetwork(network, normalizedTrainData, xDim, epochs, learningRate);

%4) ����������
testData  = ReadData('TestData.txt');
normalizedTestData = NormalizeDataWithRange(testData, maxTrainVector, minTrainVactor);
result = TestNetwork(network, normalizedTestData, xDim);%result=[data�еĽ�� ������������Ľ��]

%5) ������ȷ��
accurancy = GetAccuracny(result, yDim);
disp(['accurancy=' , num2str(accurancy)]);%best accurancy=0.88889
%�������仯ͼ
FigError(network, 1);