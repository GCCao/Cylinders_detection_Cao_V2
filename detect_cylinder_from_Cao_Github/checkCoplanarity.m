function [isCoplanar, planeParams] = checkCoplanarity(subpoints, subnormal)
    % subpoints是4x3的矩阵，表示四个点的坐标
    % subnormal是4x3的矩阵，表示四个点的法向量
    
    % 计算三个向量，由四个点生成
    v1 = subpoints(2,:) - subpoints(1,:);
    v2 = subpoints(3,:) - subpoints(1,:);
    v3 = subpoints(4,:) - subpoints(1,:);
    
    % 计算v1和v2的叉积，得到平面的法向量
    normal = cross(v1, v2);
    
    % 计算v3在这个法向量上的投影，通过点积判断是否为0
    isCoplanar = abs(dot(normal, v3)) < 0.1;

    % 如果共面，计算平面方程的参数
    if isCoplanar
        % 检查所有法向量之间的夹角
        referenceNormal = subnormal(1,:); % 选择第一个法向量作为参考
        tolerance = cosd(10); % 10度的余弦值
        
        for i = 2:4
            cosTheta = dot(referenceNormal, subnormal(i,:)) / (norm(referenceNormal) * norm(subnormal(i,:)));
            if cosTheta < tolerance
                isCoplanar = false;
                planeParams = [];
                return;
            end
        end
        
        % 平面方程的形式为 ax + by + cz + d = 0
        % 法向量 (a, b, c)
        a = normal(1);
        b = normal(2);
        c = normal(3);
        % 计算d：使用第一个点subpoints(1,:)代入平面方程
        d = -dot(normal, subpoints(1,:));
        
        planeParams = [a, b, c, d];
    else
        % 如果不共面或法向量夹角不满足条件，返回空数组
        planeParams = [];
    end
end


% function [isCoplanar, planeParams] = checkCoplanarity(subpoints)
%     % subpoints是4x3的矩阵，表示四个点的坐标
% 
%     % 计算三个向量，由四个点生成
%     v1 = subpoints(2,:) - subpoints(1,:);
%     v2 = subpoints(3,:) - subpoints(1,:);
%     v3 = subpoints(4,:) - subpoints(1,:);
% 
%     % 计算v1和v2的叉积，得到平面的法向量
%     normal = cross(v1, v2);
% 
%     % 计算v3在这个法向量上的投影，通过点积判断是否为0
%     isCoplanar = abs(dot(normal, v3)) < 1e-6;
% 
%     % 如果共面，计算平面方程的参数
%     if isCoplanar
%         % 平面方程的形式为 ax + by + cz + d = 0
%         % 法向量 (a, b, c)
%         a = normal(1);
%         b = normal(2);
%         c = normal(3);
%         % 计算d：使用第一个点subpoints(1,:)代入平面方程
%         d = -dot(normal, subpoints(1,:));
% 
%         planeParams = [a, b, c, d];
%     else
%         % 如果不共面，返回空数组
%         planeParams = [];
%     end
% end