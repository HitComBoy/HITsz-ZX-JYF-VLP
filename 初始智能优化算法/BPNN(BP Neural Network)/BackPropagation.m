function network = BackPropagation(network, outputData, learningRate, times)
%���򴫲�+Ȩֵ����
%outputData=����������Y��������
%learningRate=ѧϰ����
%times=��ǰѧϰ����

%ÿ����Ԫ�ĸ���
xDim = length( network.inputNeure(1).weight );
inputNeureNum  = size(network.inputNeure, 2);
hiddenNeureNum = size(network.hiddenNeure, 2);
outputNeureNum = size(network.outputNeure, 2);

%���ά��
if outputNeureNum ~= length(outputData)
    disp(['error dim for outputNeureNum/outputData=', num2str(outputNeureNum), '/', num2str(length(outputData))]);
    return;
end

%1) �����
%���򴫲�
delta = zeros(1, outputNeureNum);
for outputIndex = 1: outputNeureNum
    diffY = outputData(outputIndex) - network.outputNeure(outputIndex).output;%_yi_(�������) - yi(�������)
    v_k = network.outputNeure(outputIndex).netSum;%����f'������v_1_k...v_nk_k
    delta(outputIndex) = diffY * ActiveFuncDerivative(v_k);%[3.16 a]
    network.error(times, outputIndex) = diffY;%��¼��������Ԫ�����
end
%Ȩֵ����
for outputIndex = 1: outputNeureNum
    for hiddenIndex = 1: hiddenNeureNum
        x_k_1 = network.hiddenNeure(hiddenIndex).output;%��k-1����������=����������

        w_k_s = network.outputNeure(outputIndex).weight(hiddenIndex);%Ȩֵ����
        network.outputNeure(outputIndex).weight(hiddenIndex) = w_k_s + learningRate*x_k_1*delta(outputIndex);
    end
end

%2) ������
%��ǰk=������
%���򴫲�
delta_k1 = delta;%delta(k+1) ������delta = [1xoutputNeureNum]
w_k1 = zeros(outputNeureNum, hiddenNeureNum);%W(k+1) ������Ȩֵ
for outputIndex = 1 : outputNeureNum
    for hiddenIndex = 1: hiddenNeureNum
        w_k1(outputIndex, hiddenIndex) = network.outputNeure(outputIndex).weight(hiddenIndex);
    end
end
f_k = zeros(hiddenNeureNum, hiddenNeureNum);%F(k) ��������ݶ�
for hiddenIndex = 1: hiddenNeureNum
    v_k = network.hiddenNeure(hiddenIndex).netSum;
    f_k(hiddenIndex, hiddenIndex) = ActiveFuncDerivative(v_k);
end
delta_k = delta_k1 * w_k1 * f_k;%���򴫲����㹫ʽdelta(k) = delta(k+1)W(k+1)F(k) = [1xhiddenNeureNum]
%Ȩֵ����
w_k_s = zeros(hiddenNeureNum, inputNeureNum);%W(k)(s) ������Ȩֵ
for hiddenIndex = 1: hiddenNeureNum
    for inputIndex = 1: inputNeureNum
        w_k_s(hiddenIndex, inputIndex) = network.hiddenNeure(hiddenIndex).weight(inputIndex);
    end
end
x_k_1 = zeros(inputNeureNum, 1);%��k-1����������=��������� X(k-1)
for i = inputIndex: inputNeureNum
    x_k_1(inputIndex) = network.inputNeure(inputIndex).output;
end
w_k_s1 = w_k_s + learningRate * (x_k_1*delta_k)';%Ȩֵ���¹�ʽW(k)(s+1) = W(k)(s) + LR*(X(k-1)*delta(k))'
for inputIndex = 1: inputNeureNum
    for hiddenIndex = 1: hiddenNeureNum
        network.hiddenNeure(hiddenIndex).weight(inputIndex) = w_k_s1(hiddenIndex, inputIndex);
    end
end

%3) �����
%���򴫲�
delta_k1 = delta_k;%delta(k+1) �������delta = [1xhiddenNeureNum]
w_k1 = zeros(hiddenNeureNum, inputNeureNum);%W(k+1) �������Ȩֵ
for hiddenIndex = 1 : hiddenNeureNum
    for inputIndex = 1: inputNeureNum
        w_k1(hiddenIndex, inputIndex) = network.hiddenNeure(hiddenIndex).weight(inputIndex);
    end
end
f_k = zeros(inputNeureNum, inputNeureNum);%F(k) �������ݶ�
for inputIndex = 1: inputNeureNum
    v_k = network.inputNeure(inputIndex).netSum;
    f_k(inputIndex, inputIndex) = ActiveFuncDerivative(v_k);
end
delta_k = delta_k1 * w_k1 * f_k;%���򴫲����㹫ʽdelta(k) = delta(k+1)W(k+1)F(k) = [1xinputNeureNum]
%Ȩֵ����
w_k_s = zeros(inputNeureNum, xDim);%W(k)(s) �����Ȩֵ
for inputIndex = 1: inputNeureNum
    for xIndex = 1: xDim
        w_k_s(inputIndex, xIndex) = network.inputNeure(inputIndex).weight(xIndex);
    end
end
x_k_1 = zeros(xDim, 1);%��k-1����������=���������� X(k-1)
for i = xIndex: xDim
    x_k_1(xIndex) = network.inputNeure(1).input(xIndex);
end
w_k_s1 = w_k_s + learningRate * (x_k_1*delta_k)';%Ȩֵ���¹�ʽW(k)(s+1) = W(k)(s) + LR*(X(k-1)*delta(k))'
for inputIndex = 1: inputNeureNum
    for xIndex = 1: xDim
        network.inputNeure(inputIndex).weight(xIndex) = w_k_s1(inputIndex, xIndex);
    end
end

end

