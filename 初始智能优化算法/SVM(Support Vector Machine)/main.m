%Ŀ��: ʹ��֧���������㷨 ���Χ������
%Χ������������� Χ������.jpg �� http://www.cnblogs.com/end/p/3848740.html
clc; clear; close all;

%Step1=ָ��SVM�Ĳ���
C = 10;%��������ĳͷ��̶�
kernelType = 'RBF';%�˺��������� ����ʹ�þ��������RBF
delta = 5;%RBF�еĲ���
%��ͼ�Ĳ���
line_width=2;
point_width=15;
minX=-1; maxX=7;
minY=-1; maxY=8;

%Step2=��ʼ��Χ������
%��
x_cows = [2,5 ; 3,2 ; 3,3 ; 3,4 ; 4,1 ; 4,2 ; 4,4; 5,2 ; 5,4];%������� ÿ��=һͷ���x��y
rowNum = size(x_cows, 1);
y_cows = ones(rowNum, 1);%������ ��ķ����Ϊ1
%��
x_wolves = [0,0 ; 0,2 ; 1,1 ; 2,0 ; 2,7; 4,0 ; 6,3 ; 6,4];%�ǵ����� ÿ��=һͷ�ǵ�x��y
rowNum = size(x_wolves, 1);
y_wolves = -ones(rowNum, 1);%������ �ǵķ����Ϊ-1
%�������ǵķֲ����
figure;
hold on;
plot(x_cows(:,1),   x_cows(:,2),   'k.', 'MarkerSize', point_width);
plot(x_wolves(:,1), x_wolves(:,2), 'rx', 'MarkerSize', point_width);
axis( [minX maxX minY maxY] );

%Step3=ʹ��SVM���Χ������
%1) ׼��SVMģ�͵���������(��ѵ������)
% X=[x1,x2]
% Y=[y]
X = [x_cows', x_wolves'];%dim*n���� nΪ�������� dimΪX��ά��(Χ������dim=2)
Y = [y_cows', y_wolves'];%1*n���� nΪ�������� ֵΪ+1��-1

%2) SVMѵ�� ���֧������
supportVector = SVMTrain(X, Y, kernelType, C, delta);
%��֧�����õ�����(֧������) �ú�Ȧ��ע
plot(supportVector.X(1,:), supportVector.X(2,:), 'ro', 'MarkerSize', point_width);

%3) SVM��������ƽ��
[x1, x2] = meshgrid(minX:0.05:maxX, minY:0.05:maxY);%���������
[rows, cols] = size(x1);
length = rows*cols;
Xtest = [reshape(x1,1,length); reshape(x2,1,length)];%ÿ�������� ת��Ϊ һ��������=dim*nn
Ytest = SVMTest(supportVector, Xtest, kernelType, delta);%1*nn ��Ӧ������ķ�����
%�ȸ��߻���(�õȸ��߼�ΪΧ��)
Ygrid = reshape(Ytest, rows, cols);
contour(x1, x2, Ygrid, 'm');%�ȸ���