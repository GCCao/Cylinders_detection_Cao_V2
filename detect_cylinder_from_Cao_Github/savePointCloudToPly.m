function savePointCloudToPly(points, filename)
    % 确保输入的点云数据是一个Nx3的数组
    if size(points, 2) ~= 3
        error('点云数据必须是一个 Nx3 的数组');
    end

    % 打开文件
    fid = fopen(filename, 'w');
    if fid == -1
        error('无法创建文件');
    end

    % 写入Ply文件头
    fprintf(fid, 'ply\n');
    fprintf(fid, 'format ascii 1.0\n');
    fprintf(fid, 'element vertex %d\n', size(points, 1));
    fprintf(fid, 'property float x\n');
    fprintf(fid, 'property float y\n');
    fprintf(fid, 'property float z\n');
    fprintf(fid, 'end_header\n');

    % 写入点云数据
    for i = 1:size(points, 1)
        fprintf(fid, '%.6f %.6f %.6f\n', points(i, 1), points(i, 2), points(i, 3));
    end

    % 关闭文件
    fclose(fid);

    fprintf('点云数据已保存到 %s\n', filename);
end