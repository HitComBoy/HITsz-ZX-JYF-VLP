function sumLen = GetLen( position , point )
%��ȡ ��ǰ��position �� ����n����point �ľ����ƽ����

x = position(1);
y = position(2);
    
sumLen = 0;
n = size(point, 1);
for i = 1: 1: n
    px = point(i, 1);
    py = point(i, 2);
    length = (px-x)^2 + (py-y)^2;
    
    sumLen = sumLen + length;
end

end

