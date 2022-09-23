%Ŀ��: ʹ��ģ���˻���������������
%�����: �õ㵽ָ��n����ľ���֮����С
clc;clear all;close all;

%Step1=ָ��SA�Ĳ���
step=1.5;%�ֲ������Ĳ���
k=2;%�¶�ϵ��

%Step2=��ʼ�����������
%���ָ����ά�����ĵ�([0~100, 0~100])---�����Ӧ�����⸽��
randCenter = rand(1, 2) * 100;
%ָ��n=50�������point �ֲ���Center������30��Χ��
pointX = randCenter(1)-30 + rand(50,1).*60;
pointY = randCenter(2)-30 + rand(50,1).*60;
point = [pointX pointY];%ÿ��=һ����
%��������ĳ�ʼ���
figure(1);
plot(point(:,1), point(:,2), '*');%n�������
hold on;
plot(randCenter(1) , randCenter(2) , 'k*');%�����Ĵ���λ��

%Step3=ʹ��SA������������
%���ָ��һ����ʼ�ķ����
FermatPoint = rand(1,2) .* 100;

%׼��avi����
aviObj=VideoWriter('Simulated Annealing.avi');%����Ϊavi
aviObj.FrameRate=30;
open(aviObj);

for T = 3000: -10: 1%ѭ��ģ�⽵�µĹ���
    %1) i+1���λ��
    dx = -step + 2*step*rand(1,1);%-step~step
    dy = -step + 2*step*rand(1,1);%-step~step
    nextPoint = [FermatPoint(1)+dx FermatPoint(2)+dy];
    
    %2) i+1���i�� ��ָ��n����ľ���֮��
    nextLen   = GetLen(nextPoint, point);
    FermatLen = GetLen(FermatPoint, point);
    
    %3) SAģ������˻�Ĺ���
    if nextLen <= FermatLen%�� i+1������� <= i������� (�˻� ��������)
        FermatPoint = nextPoint;%���ܸ��ƶ�
    else%��i+1�����������
        %��һ���ĸ���P�����ƶ� �� �����������ʱ��T�����ƶ��𽥽���
        dE = nextLen-FermatLen;%������
        P = exp(dE/k/T);%����ѧ����
        if rand(1,1) > P%�����˶�
            FermatPoint=nextPoint;
        end
    end
    
    %���ӻ���ǰ���
    plot(FermatPoint(1) , FermatPoint(2), 'ro');
    length = GetLen(FermatPoint,point);
    disp(['length=' , num2str(length)]);
    %д��avi����
    writeVideo(aviObj,getframe);
end
close(aviObj);%����avi����

%��ʾ�Ż���ķ������
figure(2);
plot(point(:,1), point(:,2), '*');
hold on;
plot(randCenter(1) , randCenter(2) , 'k*');
plot(FermatPoint(1) , FermatPoint(2), 'ro');
length = GetLen(FermatPoint,point);
disp(['length of Fermat Point=' , num2str(length)]);