function accurancy = GetAccuracny(result, yDim)
%����������ȷ��
%result = [���۽��, ��������, ��������������]

rightCnt = 0;
totalSample = size(result,1);
for sampleIndex = 1: totalSample
    for yIndex = 1: yDim
        %���۽����λ��=1 �� ��Ԫ��������λ��=1
        if result(sampleIndex, yIndex)==1 && result(sampleIndex, yIndex+2*yDim)==1
            rightCnt = rightCnt+1;
            break;
        end
    end
end

accurancy = rightCnt/totalSample;

end

