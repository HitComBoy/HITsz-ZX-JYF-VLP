function bStar = GetBstar(supportVector, kernelType, delta)
%���������ֵbStar

%��reference�е�Eq. 3-17
%sgn{(w*x)-b} չ�����xi=x
wx = (supportVector.alpha' .* supportVector.Y) * KernelFunc(supportVector.X, supportVector.X, kernelType, delta);%1*nPart
b = wx - supportVector.Y;%1*nPart

%����������һ��֧�������ľ�ֵ
nPart = size(b, 2);
for i=1: nPart
    if supportVector.Y(i) > 0
        index1 = i;
        break;
    end
end
for i=1: nPart
    if supportVector.Y(i) < 0
        index2 = i;
        break;
    end
end
bStar = (b(index1) + b(index2)) / 2;%1*1

end

