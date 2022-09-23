function fitness = Lambertian( position )
%输入坐标x和y 输出适应值
%函数值z=适应度值

x = position(1);
y = position(2);
z = position(3);
	  
fitness = FuncCalculate(x, y);

end

