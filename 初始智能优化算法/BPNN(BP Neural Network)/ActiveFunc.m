function y = ActiveFunc( x )
%������Ԫ�ļ����
%������ʹ��˫��S�κ��� [-1,1]

alpha = 2;

y = 2 / (1+exp(-alpha*x)) - 1;

end

