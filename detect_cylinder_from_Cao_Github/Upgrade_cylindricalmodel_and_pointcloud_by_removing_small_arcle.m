function [detectedCylinders,detectedCylinders_pointCloud,indices_in_P] = Upgrade_cylindricalmodel_and_pointcloud_by_removing_small_arclength_ratio(detectedCylinders,detectedCylinders_pointCloud,arc_length_ratio_proj)
% 净化点云，更新圆柱的参数。有一些处于弯曲部分的点，但是满足距离和法向约束要求，被误以为是圆柱体上的点，应当被删除
%                             这类点的特点是，在圆弧上的占比比较小，通常少于1/5

indices_in_P ={};

for i=1:size(detectedCylinders,2)

    P = detectedCylinders_pointCloud{i};
    Cylinder = detectedCylinders{i};

    A = Cylinder.Parameters(1:3);
    B = Cylinder.Parameters(4:6);

    AB = B - A;  
    AB_norm = norm(AB);  

    % i
    % 
    % if size(P,2) ==0
    %     ee=0;
    % end
    % 
    % if size(P,2) ~= size(A,2)
    %     ee=0;
    % end
    % 
    % 如果 AB_norm 为零，返回 NaN 或者原始点云  
    if AB_norm == 0  
        error('A 和 B 不能是相同的点');  
    end  

    


    %圆柱的直径
    R = Cylinder.Parameters(7);

    %The arc length ratio of the projected point cloud on the circumference
    % %投影弧长占比的阈值，低于这个阈值，认为这部分点云大概率不属于圆柱
    % arc_length_ratio_proj = 0.25;

    %计算P在AB直线上的投影，注意：P_proj中最远的两个点未必就是A和B,因为这里的A和B来自圆柱体模型的端点，而P是采用额外的方法获得圆柱体表面点
    P_proj = projectPointsToLine_by_MatrixOperation(P, A, B);

    %求P_proj中最远的两个点。如果直接计算两两的距离，非常耗时，因为P_proj点数可能很大
    % [C, D, maxDistance] = findFarthestPoints_inP_proj(P_proj, A, B);
    [C, D, maxDistance] = findFarthestPoints_inP_proj_new(P_proj);  
    

    segments = segmentPointCloud(P, C, D);


    % pcshow(P,[1,0.5,0.5],'MarkerSize',50); %显示点云数据

    %计算点云数量最大的分段及数量值
    maxnum = 0;
    for j=1:length(segments)
        PPP = segments{j};
        n  = size(PPP,1);
        if n>maxnum
            maxnum = n;
        end
    end


    %小圆柱上至少应该具备的点云数量
    min_numpoints = maxnum*arc_length_ratio_proj;

    %找到满足点数要求的小圆柱,必将它们汇集成有效的圆柱点
    Points_effective_arc_length = [];
    for j=1:length(segments)
        PP = segments{j};
        n  = size(PP,1);
        if n>min_numpoints
            Points_effective_arc_length = [Points_effective_arc_length;PP];
        end
    end

    %-----------对圆柱模型进行更新，重新计算点云的首尾点,赋值给圆柱    
    P_proj = projectPointsToLine_by_MatrixOperation(Points_effective_arc_length, A, B);    
    [C, D, maxDistance] = findFarthestPoints_inP_proj_new(P_proj); 
    para =[C D R];
    model = cylinderModel(para);      
    detectedCylinders{i} = model;

    %对圆柱的点云进行更新
    detectedCylinders_pointCloud{i} = Points_effective_arc_length;


    %获取当前圆柱点云的索引,是升级以后的点云在P中的索引
    % tic
    % indices_in_P{i} = find_indices_kdtree(P, Points_effective_arc_length);
    % toc
    % 使用 ismember 函数找出 B 中每个点在 P_proj 中的索引
    % tic
    [~, indices] = ismember(Points_effective_arc_length, P, 'rows');
    indices_in_P{i} =  indices;
    % toc

    



    % hold on
    % pcshow(Points_effective_arc_length,[0.5,1,0.5],'MarkerSize',50); %显示点云数据
end


end
