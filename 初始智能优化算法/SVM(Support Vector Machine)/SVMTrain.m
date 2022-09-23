function supportVector = SVMTrain(X, Y, kernelType, C, delta)
%����X��Y��ѵ������ ��SVMģ�ͽ���ѵ��

%X = dim*(num=n)
%Y = 1*n

%MyQuadProg������Ż�����
alpha = MyQuadProg(X, Y, kernelType, C, delta);%alpha=n*1�ľ���

%����SVM�����ּ��ϵͳ�о� ��� ��ʽ
epsilon = 1e-8;%С��
validIndex = find( abs(alpha)>epsilon );%�����Ч��֧������������(alpha>0�Ķ�Ӧ�±�����)

supportVector.alpha = alpha(validIndex);%nPart*1
supportVector.X     = X(:, validIndex);%ȡ��alpha>0����
supportVector.Y     = Y(validIndex);

end

