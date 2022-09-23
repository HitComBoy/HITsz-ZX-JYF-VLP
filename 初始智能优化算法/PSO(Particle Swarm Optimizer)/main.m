%Ŀ��: ʹ������ȺPSO�Ż��㷨 �����ά���������ֵ
clc;clear all;close all;

%Step1=ָ��PSO�Ĳ���
num = 64;%��Ⱥ�и�����Ŀ
c1 = 2; c2 = 2;%c1 c2=ѧϰ����(�Ǹ�����)
w = 0.8;%��������(�Ǹ���)
alpha = 0.4;%Լ������=�����ٶȵ�Ȩ��
v_limit = 0.2;%�ٶȼ�ֵ

%Step2=��ʼ�����ĺ���fun
[fun_x, fun_y, fun_z] = peaks;
figure(1);
hold on;
meshc(fun_x, fun_y, fun_z);

%Step3=ʹ��PSO��⺯�������ֵ
particle = zeros(num, 5);%һȺ���� ÿ��=[��� λ��x λ��y �ٶ�x �ٶ�y]
pi = zeros(num, 3);%������ʷ���� ÿ��=[��� λ��x λ��y] (ÿ���������һ����ʷ���� �� ��ʷ���Ŵ��м���=����Ϊֹ�����ţ�������ȥ��״̬)
pg = zeros(1, 2);%Ⱥ����ʷ���� ÿ��=[λ��x λ��y] (��������Ⱥ����һ��)
pg(1) = -3 + 6*rand(1,1);%x=-3~3
pg(2) = -3 + 6*rand(1,1);%y=-3~3
%��������Ⱥ��ֵ(ÿ���������λ�ú��ٶ�)
for i = 1: 1: num
    %ÿ������
    particle(i,1) = i;%���
    particle(i,2) = -3+6*rand(1,1);%x=-3~3
    particle(i,3) = -3+6*rand(1,1);%y=-3~3
    particle(i,4) = -v_limit+v_limit*2*rand(1,1);%vx=-v_limit~v_limit
    particle(i,5) = -v_limit+v_limit*2*rand(1,1);%vy=-v_limit~v_limit
    %������ʷ����
    pi(i,1) = particle(i,1);%���
    pi(i,2) = particle(i,2);%x
    pi(i,3) = particle(i,3);%y
    %Ⱥ����ʷ���ų�ֵ
    if GetFitness( particle(i,2:3) ) > GetFitness( pg )
        pg = particle(i,2:3);
    end
end
%��ʼ�������ʾ
figure(1);
hold on;
plot_x = particle(:,2);
plot_y = particle(:,3);
plot_z = FuncCalculate(plot_x, plot_y);
plot3(plot_x , plot_y , plot_z , '*r');%����λ��
axis([-3,3 , -3,3]);
hold off;

%׼��avi����
aviObj = VideoWriter('Particle Swarm Optimizer.avi');%����Ϊavi
aviObj.FrameRate = 30;
open(aviObj);

for cycle=1:1:100%����Ⱥ�˶�����
    tmp_pg = pg;%Ⱥ������
	%ÿ�����ӵ��˶�Ӧ���ǲ��е� ����ˢ�µ�Ⱥ����ʷ����Ӧ������Ӱ����һ�ֵ��˶�
	%����ÿ������ PSO�˶�ʱ����pg
	%���������˶������� ��ȥˢ��pg
	
    for i = 1: 1: num%ÿ������
        %1) ���Ӳ���(��: ����Ⱥ�Ż��㷨 ���,����,������,����ƽ ��ʽ3��4)
        v_id = particle(i,4:5);%i���ӵ��ٶ�=(v_x v_y)
        x_id = particle(i,2:3);%i���ӵ�λ��=(x_x x_y)
        p_id = pi(i, 2:3);%i���ӵĸ�����ʷ����λ��
        
        r1 = rand(1,1);%r1,r2�ǽ���[0,1]֮��������
        r2 = rand(1,1);
        
        v_id = w*v_id + c1*r1*(p_id-x_id) + c2*r2*(pg-x_id);
        x_id = x_id + alpha*v_id;
        
        %2) ����Խ������ӽ��е���: ����߽���=�ٶȴ�С���� ����ȡ��
        if ( x_id(1)>3 && v_id(1)>0 ) || ( x_id(1)<-3 && v_id(1)<0 )%x+��x-����Խ��
            x_id = x_id - alpha*v_id;%�ָ�֮ǰ��λ��
            v_id(1) = -v_id(1);%x�������ٶ�ȡ��
            x_id = x_id + alpha*v_id;%Ȼ�����˶�
        end
        if ( x_id(2)>3 && v_id(2)>0 ) || ( x_id(2)<-3 && v_id(2)<0 )%y+��y-����Խ��
            x_id = x_id - alpha*v_id;%�ָ�֮ǰ��λ��
            v_id(2) = -v_id(2);%y�������ٶ�ȡ��
            x_id = x_id + alpha*v_id;%Ȼ�����˶�
        end
        
        %3) �����˶�
		%ˢ�����ӵ�λ�ú��ٶ�
        particle(i,2:3) = x_id;
        particle(i,4:5) = v_id;
        %ˢ�¸�����ʷ����
        if GetFitness( particle(i,2:3) ) > GetFitness( pi(i,2:3) )
            pi(i,2:3) = particle(i,2:3);
        end
        %ˢ��Ⱥ����ʷ����
        if GetFitness( particle(i,2:3) ) > GetFitness( tmp_pg )
            tmp_pg = particle(i,2:3);%tmp_pg ��Ӱ����һ�ֺ����������ж�
        end
    end
    pg=tmp_pg;
    
	%���ӻ���ǰ���
    figure(2);
	%����ͼ��
    meshc(fun_x, fun_y, fun_z);
    hold on;
    %ÿ�����ӵķֲ�
    plot_x = particle(:,2);
    plot_y = particle(:,3);
    plot_z = FuncCalculate(plot_x, plot_y);
    plot3( plot_x , plot_y , plot_z , '*r');%����λ��
    axis([-3 3 -3 3]);
    hold off;
    %д��avi����
    getframe
    frame = ans.cdata;
    frame = frame(1:343,1:435);
    writeVideo(aviObj,frame);
end
%����avi����
close(aviObj);



%���Ľ����ʾ
figure(3);
meshc(fun_x, fun_y, fun_z);%����ͼ��
hold on;
plot_x=particle(:,2);
plot_y=particle(:,3);
plot_z=FuncCalculate(plot_x, plot_y);
plot3( plot_x , plot_y , plot_z , '*r');%����λ��
axis([-3,3 , -3,3]);
hold off;