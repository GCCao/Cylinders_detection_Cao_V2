function pointCloudData = pclread_own(filename)
    % 打开文件
    fid = fopen(filename, 'rb');
    if fid == -1
        error('无法打开文件 %s', filename);
    end
    
    % 读取点的数量和模式
    numPoints = fread(fid, 1, 'uint64');
    mode = fread(fid, 1, 'uint64');
    
    % 定义基本的x, y, z读取格式
    formatSpec = {'float32', 'float32', 'float32'}; % x, y, z
    
    % 根据mode添加相应的字段
    if bitand(mode, 1)
        formatSpec = [formatSpec, {'float32', 'float32', 'float32'}]; % r, g, b
    end
    if bitand(mode, 2)
        formatSpec = [formatSpec, {'float32'}]; % intensity
    end
    if bitand(mode, 4)
        formatSpec = [formatSpec, {'float32', 'float32', 'float32'}]; % nx, ny, nz
    end
    if bitand(mode, 8)
        formatSpec = [formatSpec, {'float32'}]; % confidence
    end
    if bitand(mode, 16)
        formatSpec = [formatSpec, {'float32'}]; % curvature
    end
    
    % 构建读取格式字符串
    formatString = strjoin(formatSpec, '=>%f');
    
    % 初始化结果数组
    pointCloudData = zeros(numPoints, numel(formatSpec));
    
    % 读取每个点的数据
    for i = 1:numPoints
        pointData = fread(fid, numel(formatSpec), formatString);
        pointCloudData(i, :) = pointData;
    end
    
    % 关闭文件
    fclose(fid);
end