function fitness = GetFitness( position )
%��������x��y �����Ӧֵ
%����ֵz=��Ӧ��ֵ

x=position(1);
y=position(2);
	  
fitness = FuncCalculate(x, y);

end

