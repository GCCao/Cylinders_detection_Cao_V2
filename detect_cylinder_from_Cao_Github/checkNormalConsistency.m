function [isRealPlane,angle_std] = checkNormalConsistency(points, normals, Thstd)  
    % 计算法向量的变化  
    % 对于真实平面，法向量应该高度一致  
    % 对于圆柱体，法向量会有渐变  
    
    % 1. 计算法向量之间的夹角  
    n = size(points, 1);  
    angles = zeros(n, 1);  
    mean_normal = mean(normals, 1);  
    mean_normal = mean_normal / norm(mean_normal);  
    
    for i = 1:n  
        angles(i) = acos(abs(dot(normals(i,:), mean_normal)));  
    end  
    
    % 2. 分析角度分布  
    angle_std = std(angles);  
    
    % 3. 判断标准  
    if angle_std < Thstd  % 阈值需要根据实际情况调整  
        isRealPlane = true;  
    else  
        isRealPlane = false;  
    end  
end