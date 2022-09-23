function Kernel = KernelFunc(Xi ,X, kernelType, delta)
%�˺����ļ���

%Xi=dim*num(����num��Xiһ������)
%X=dim*n
%Kernel=num*n

%��ͬ����ĺ˺���
switch kernelType
case 'linear'
    Kernel = Xi' * X;
case 'RBF'
    num = size(Xi, 2);
    n   = size(X,  2);
    
    %����|x-xi|^2�ľ�����ʽ
    X2 = sum(X' .* X', 2);%ÿ����� n*1
    Xi2 = sum(Xi' .* Xi', 2);%ÿ����� num*1
    XiX = Xi'*X;%num*n
    tmp = abs( repmat(Xi2,[1 n]) + repmat(X2', [num 1]) - 2*XiX );%num*n
    
    delta2 = delta^2;
    Kernel = exp(-tmp ./ delta2);
end

end

