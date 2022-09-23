%Ŀ��: ʹ�û����㷨 ��⺯�������ֵ
clc; clear all; close all; clf;

%Step1=ָ��GA�Ĳ���
num = 256;%��������

%Step2=��ʼ�����ĺ���fun
fun_x = 15: 0.1: 25;%x=15~25
fun_y = fun_x + 10* sin(5* fun_x)+ 7* cos(4* fun_x);
figure(1);
hold on;
plot(fun_x, fun_y, '-b');

%Step3=ʹ��GA��⺯�������ֵ
%�������һȺ����
individual = zeros(num, 2);
for i = 1: 1: num
    individual(i,1) = i;%����ı��
    individual(i,2) = 15 + 10*rand(1, 1);%�ø������ڵ�x=15~25
end

%׼��avi����
aviObj=VideoWriter('Genetic Algorithm.avi');%����Ϊavi
aviObj.FrameRate=30;
open(aviObj);

for cycle = 1: 1: 100%Ⱥ�己ֳ��ѭ������
    %1) ����ÿ���������Ӧ��ֵ
    fitness = GetFitness(individual);
    
    %2) ��ȡĸ����(��Ӧ�ȴ� �ϴ���ʱ�ѡΪĸ��)
    parent = zeros(num/2, 2);
    for i= 1: 1: num/2
        parent(i,:) = GetTwoParents(fitness);%ÿ��=����ĸ���ı��
    end
    
    %3) ���� �����Ӵ�
    tmpParent = zeros(num, 2);
    for i = 1: 1: num/2
        genesOfTwoChildren = CrossParent( parent(i,:), individual );%����������=�����Ӵ��Ļ��� 
        geneOfChild1 = genesOfTwoChildren(1:12);%12λ�Ļ���
        geneOfChild2 = genesOfTwoChildren(13:24);%12λ�Ļ���
        
		%�����Ӵ�
        tmpParent(2*i-1, 1) = 2*i-1;%���
        tmpParent(2*i-1, 2) = gene2num( geneOfChild1 );%�ø������ڵ�x
        tmpParent( 2*i,  1) = 2*i;%���
        tmpParent( 2*i,  2) = gene2num( geneOfChild2 );%�ø������ڵ�x
    end
	%ˢ��һ����Ա
    individual=tmpParent;
    
    %���ӻ���ǰ���
    %ԭ����
    fun_x= 15: 0.1: 25;%15~25
    fun_y= fun_x + 10* sin(5* fun_x)+ 7* cos(4* fun_x);
    figure(1);
    plot(fun_x, fun_y, '-b');
    hold on;
    %�����
    fun_x= individual(:,2);%�������ڵ�x
    fun_y= fun_x + 10* sin(5* fun_x)+ 7* cos(4* fun_x);
    plot(fun_x, fun_y, '*r');
    hold off;
    %д��avi����
    writeVideo(aviObj, getframe);
end
%����avi����
close(aviObj);