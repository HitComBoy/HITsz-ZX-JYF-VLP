%目的: 使用粒子群PSO优化算法 求解三维函数的最大值
clc
clear 
close all

%Step2=初始化求解的函数fun
[fun_x, fun_y, fun_z] = peaks;

figure(1);
meshc(fun_x, fun_y, fun_z);%函数图像
hold on;


plot_x=-1.875;
plot_y=-1.875;
plot_z=FuncCalculate(plot_x, plot_y)+0.3;
f1=plot3( plot_x , plot_y , plot_z , '.','Color','#A2142F','MarkerSize',22);%粒子位置
hold on;
plot_x=-0.75;
plot_y=-0.875;
plot_z=FuncCalculate(plot_x, plot_y)+0.3;
f2=plot3( plot_x , plot_y , plot_z , '.','Color','#0072BD','MarkerSize',22);%粒子位置
hold on;
quiver3(-1.875,-1.875,FuncCalculate(-1.875, -1.875)+0.3,-0.75+1.875,-0.875+1.875,FuncCalculate(-0.75, -0.875)-FuncCalculate(-1.875, -1.875),0.95,'Color','#D95319','LineWidth',1.2)
hold on;
plot_x=0;
plot_y=1.625;
plot_z=FuncCalculate(plot_x, plot_y)+0.3;
f3=plot3( plot_x , plot_y , plot_z , '.','Color','#77AC30','MarkerSize',22);%粒子位置
hold on;
quiver3(-0.75,-0.875,FuncCalculate(-0.75,-0.875)+0.3,0+0.75,1.625+0.875,FuncCalculate(0, 1.625)-FuncCalculate(-0.75, -0.875),0.95,'Color','#D95319','LineWidth',1.2)
hold on;


plot_x=2.5;
plot_y=-2.5;
plot_z=FuncCalculate(plot_x, plot_y)+0.3;
plot3( plot_x , plot_y , plot_z , '.','Color','#A2142F','MarkerSize',22);%粒子位置
hold on;
plot_x=1.375;
plot_y=0.125;
plot_z=FuncCalculate(plot_x, plot_y)+0.3;
plot3( plot_x , plot_y , plot_z , '.','Color','#0072BD','MarkerSize',22);%粒子位置
hold on;
quiver3(2.5,-2.5,FuncCalculate(2.5, -2.5)+0.3,1.375-2.5,0.125+2.5,FuncCalculate(1.375, 0.125)-FuncCalculate(2.5, -2.5),0.95,'Color','#D95319','LineWidth',1.2)
hold on;
plot_x=0;
plot_y=1.625;
plot_z=FuncCalculate(plot_x, plot_y)+0.3;
plot3( plot_x , plot_y , plot_z , '.','Color','#77AC30','MarkerSize',22);%粒子位置
hold on;
quiver3(1.375,0.125,FuncCalculate(1.375,0.125)+0.3,0-1.375,1.625-0.125,FuncCalculate(0, 1.625)-FuncCalculate(1.1375, 0.125),0.95,'Color','#D95319','LineWidth',1.2)
hold on;


plot_x=0.25;
plot_y=-1.125;
plot_z=FuncCalculate(plot_x, plot_y)+0.3;
plot3( plot_x , plot_y , plot_z , '.','Color','#A2142F','MarkerSize',22);%粒子位置
hold on;
plot_x=0.875;
plot_y=-0.5;
plot_z=FuncCalculate(plot_x, plot_y)+0.3;
plot3( plot_x , plot_y , plot_z , '.','Color','#0072BD','MarkerSize',22);%粒子位置
hold on;
quiver3(0.25,-1.125,FuncCalculate(0.25, -1.125)+0.3,0.875-0.25,-0.5+1.125,FuncCalculate(0.875, -0.5)-FuncCalculate(0.25, -1.125),0.95,'Color','#D95319','LineWidth',1.2)
hold on;
plot_x=0;
plot_y=1.625;
plot_z=FuncCalculate(plot_x, plot_y)+0.3;
plot3( plot_x , plot_y , plot_z , '.','Color','#77AC30','MarkerSize',22);%粒子位置
hold on;
quiver3(0.875,-0.5,FuncCalculate(0.875,-0.5)+0.3,0-0.875,1.625+0.5,FuncCalculate(0, 1.625)-FuncCalculate(0.875, -0.5),0.95,'Color','#D95319','LineWidth',1.2)
hold on;



plot_x=-2.625;
plot_y=1.625;
plot_z=FuncCalculate(plot_x, plot_y)+0.3;
plot3( plot_x , plot_y , plot_z , '.','Color','#A2142F','MarkerSize',22);%粒子位置
hold on;
plot_x=-0.375;
plot_y=1.55;
plot_z=FuncCalculate(plot_x, plot_y)+0.5;
plot3( plot_x , plot_y , plot_z , '.','Color','#0072BD','MarkerSize',22);%粒子位置
hold on;
quiver3(-2.625,1.625,FuncCalculate(-2.625, 1.625)+0.3,-0.375+2.625,1.55-1.625,FuncCalculate(-0.375, 1.55)-FuncCalculate(-2.625, 1.625)+0.2,0.95,'Color','#D95319','LineWidth',1.2)
hold on;
plot_x=0;
plot_y=1.625;
plot_z=FuncCalculate(plot_x, plot_y)+0.3;
plot3( plot_x , plot_y , plot_z , '.','Color','#77AC30','MarkerSize',22);%粒子位置
hold on;
quiver3(-0.375,1.55,FuncCalculate(-0.375,1.55)+0.5,0+0.375,1.625-1.55,FuncCalculate(0, 1.625)-FuncCalculate(-0.375, 1.55)-0.2,0.95,'Color','#D95319','LineWidth',1.2)
axis([-3,3 , -3,3]);
hold off;
zlabel('Adaptive Value')
legend([f1 f2 f3],{'Initial Particles','After one Cycle','After all Cycle'})
