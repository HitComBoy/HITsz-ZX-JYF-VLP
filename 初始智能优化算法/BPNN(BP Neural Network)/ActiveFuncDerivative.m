function dy = ActiveFuncDerivative( x )
%������Ԫ�ļ�����ĵ���
%������ʹ��˫��S�κ���

alpha = 2;
dy = alpha  * (1-ActiveFunc(x)^2) / 2;

end

