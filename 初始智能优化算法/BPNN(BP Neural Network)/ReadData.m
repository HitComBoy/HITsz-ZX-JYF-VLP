function data = ReadData(filename)
%���ļ��и�ʽ����ȡ����

fid = fopen(filename);
cellData = textscan(fid, '%f%f%f%f%f');
fclose(fid);

data = cell2mat(cellData);%=[x1,x2,x3,x4, y]

end
