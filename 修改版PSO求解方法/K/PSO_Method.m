function Calculated_point = PSO_Method(Receive_power, P_led, N_pd, N_led, m, M, Ar, num, num_cycle)
%*********************************************************************************************%
% Use the Particle Swarm Optimization(PSO) to calculate the position result in our method one.%
%*********************************************************************************************%
%*******************************Basical Parameter Setting*************************************%
c1 = 2; c2 = 2;%c1 c2=学习因子(非负常数)
w = 0.8;%惯性因子(非负数)
alpha = 0.4;%约束因子=控制速度的权重
v_limit = 0.2;%速度极值
%********************************Particle space spanning**************************************%
particle = zeros(num, 7);%一群粒子 每行=[编号 位置x 位置y 位置z 速度x 速度y 速度z]
pi = zeros(num, 4);%个体历史最优 每行=[编号 位置x 位置y 位置z] (每个个体均有一个历史最有 且 历史最优带有记忆=迄今为止的最优，包含过去的状态)
pg = zeros(1, 3);%群体历史最优 每行=[位置x 位置y 位置z] (整个粒子群仅有一个)
pg(1) = 0 + 3*rand(1,1);%x=0~3
pg(2) = 0 + 3*rand(1,1);%y=0~3
pg(3) = 0 + 3*rand(1,1);%z=0~3
%***********************************Particles setting*****************************************%
for i = 1: 1: num
    %每个粒子
    particle(i,1) = i;%编号
    particle(i,2) = 0+3*rand(1,1);%x=0~3
    particle(i,3) = 0+3*rand(1,1);%y=0~3
    particle(i,4) = 0+3*rand(1,1);%z=0~3
    particle(i,5) = -v_limit+v_limit*2*rand(1,1);%vx=-v_limit~v_limit
    particle(i,6) = -v_limit+v_limit*2*rand(1,1);%vy=-v_limit~v_limit
    particle(i,7) = -v_limit+v_limit*2*rand(1,1);%vz=-v_limit~v_limit
    %个体历史最优
    pi(i,1) = particle(i,1);%编号
    pi(i,2) = particle(i,2);%x
    pi(i,3) = particle(i,3);%y
    pi(i,4) = particle(i,3);%z
    %群体历史最优初值
    l_power = norm((Theory_Power(particle(i,2:4), P_led, N_pd, N_led, m, M, Ar)-Receive_power),2);%当前粒子位置下接收功率与实际接收功率之差
    g_power = norm((Theory_Power(pg, P_led, N_pd, N_led, m, M, Ar)-Receive_power),2);%当前粒子位置下接收功率与实际接收功率之差
    if  l_power < g_power
        pg = particle(i,2:4);
    end
end

%***********************************Particles Running*****************************************%
for cycle=1:1:num_cycle%粒子群运动次数
    tmp_pg = pg;%群体最优
	%每个粒子的运动应当是并行的 所以刷新的群体历史最优应当不会影响这一轮的运动
	%对于每个粒子 PSO运动时基于pg
	%所有粒子运动结束后 再去刷新pg
	
    for i = 1: 1: num%每个粒子
        %1) 粒子操作(见: 粒子群优化算法 李爱国,覃征,鲍复民,贺升平 公式3和4)
        x_id = particle(i,2:4);%i粒子的位置=(x y z)
        v_id = particle(i,5:7);%i粒子的速度=(v_x v_y v_z)
        p_id = pi(i,2:4);%i粒子的个体历史最优位置
        
        r1 = rand(1,1);%r1,r2是介于[0,1]之间的随机数
        r2 = rand(1,1);
        
        v_id = w*v_id + c1*r1*(p_id-x_id) + c2*r2*(pg-x_id);
        x_id = x_id + alpha*v_id;
        
        %2) 对于越界的粒子进行调整: 反射边界上=速度大小不变 方向取反
        if ( x_id(1)>3 && v_id(1)>0 ) || ( x_id(1)<0 && v_id(1)<0 )%x+或x-方向越界
            x_id = x_id - alpha*v_id;%恢复之前的位置
            v_id(1) = -v_id(1);%x方向上速度取反
            x_id = x_id + alpha*v_id;%然后再运动
        end
        if ( x_id(2)>3 && v_id(2)>0 ) || ( x_id(2)<0 && v_id(2)<0 )%y+或y-方向越界
            x_id = x_id - alpha*v_id;%恢复之前的位置
            v_id(2) = -v_id(2);%y方向上速度取反
            x_id = x_id + alpha*v_id;%然后再运动
        end
        if ( x_id(3)>3 && v_id(3)>0 ) || ( x_id(3)<0 && v_id(3)<0 )%z+或z-方向越界
            x_id = x_id - alpha*v_id;%恢复之前的位置
            v_id(3) = -v_id(3);%y方向上速度取反
            x_id = x_id + alpha*v_id;%然后再运动
        end
        
        %3) 粒子运动
		%刷新粒子的位置和速度
        particle(i,2:4) = x_id;
        particle(i,5:7) = v_id;
        %刷新个体历史最优
        l_power = norm((Theory_Power(particle(i,2:4), P_led, N_pd, N_led, m, M, Ar)-Receive_power),2);%当前粒子位置下接收功率与实际接收功率之差
        g_power = norm((Theory_Power(pi(i,2:4), P_led, N_pd, N_led, m, M, Ar)-Receive_power),2);%当前粒子移动过程中接收功率与实际接收功率之差的最小值
        if  l_power < g_power
            pi(i,2:4) = particle(i,2:4);
        end
        %刷新群体历史最优
        l_power = norm((Theory_Power(particle(i,2:4), P_led, N_pd, N_led, m, M, Ar)-Receive_power),2);%当前粒子位置下接收功率与实际接收功率之差
        g_power = norm((Theory_Power(tmp_pg, P_led, N_pd, N_led, m, M, Ar)-Receive_power),2);%所有粒子移动过程中接收功率与实际接收功率之差的最小值
        if l_power < g_power
            tmp_pg = particle(i,2:4);%tmp_pg 不影响这一轮后续的粒子判断
        end
    end
    pg=tmp_pg;  
end
Calculated_point = pg;
end

