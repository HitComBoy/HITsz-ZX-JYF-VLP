function alpha = MyQuadProg( X, Y, kernelType, C, delta)
%ʹ��quadprog����(matlab�Դ���) �����ι滮���� 1/2*x'*H*x + f'*x. H

%X=dim*(num=n)
%Y=1*n

n = size(X, 2);

%�㷨��ѡ�����
options = optimset;
options.LargeScale = 'off';
options.Display = 'off';

%ʹ��quadprog���������ι滮���� 1/2*x'*H*x + f'*x. H
%���ι滮Ŀ��x = SVM�е�alpha(n*1������)
% Q(alpha)��max = ���ι滮��min
%����SVM�����ּ��ϵͳ�о�_��� P39

%Eq. 3-16
%Y' * Y     = n*n�ķ���
%KernelFunc = (num=n)*n�ķ���
H=(Y' * Y) .* KernelFunc(X, X, kernelType, delta);%n*n�ķ���
f= -ones(n, 1);%n*1������

%������Ax<=b��Լ��
A=[];
b=[];

%SVM��Լ������ 1) sum(alpha_i*y_i)=0
Aeq = Y;%1*n
beq = 0;%1*1

%SVM��Լ������ 2) 0<= alpha_i <= C��������ĳͷ��̶� (Eq. 3-14)
lb = zeros(n, 1);%n*1
ub = C * ones(n, 1);%n*1

a0 = [];
alpha  = quadprog(H,f, A,b, Aeq,beq, lb,ub, a0, options);%���ι滮�Ľ��x=SVM��alpha n*1

end

