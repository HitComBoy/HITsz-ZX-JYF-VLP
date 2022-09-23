function Ytest = SVMTest(supportVector, Xtest, kernelType, delta)
%SVM����

%supportVector.alpha = nPart*1
%supportVector.X     = dim*nPart
%supportVector.Y     = 1*nPart
%Xtest               = dim*nn

bStar = GetBstar(supportVector, kernelType, delta);%1*1

%��reference�е�Eq. 3-17
%sgn{(w*x)-b} չ�����(xi*x)
wx = (supportVector.alpha' .* supportVector.Y) * KernelFunc(supportVector.X, Xtest, kernelType, delta);%1*nn

Ytest = sign(wx - bStar);%1*nn
end

