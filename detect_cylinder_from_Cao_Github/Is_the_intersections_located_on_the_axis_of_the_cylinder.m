function [Bool_intersection_valid,max_dist,max_E_ridus,mean_Intersections,R] = Is_the_intersections_located_on_the_axis_of_the_cylinder(Intersections,subpoints_proj_on_plane,ErrDisandRidus)
%计算交点的位置关系，并判断他们是否处于圆柱的轴向上
Bool_intersection_valid = 0;
distances = [];
Dist =[];
E_ridus = [];
m=1;
% 计算任意两个点之间的距离
for i = 1:size(Intersections,1)
    for j = i+1:size(Intersections,1)
        % 计算两点之间的欧氏距离
        dist = sqrt(sum((Intersections(i,:) - Intersections(j,:)).^2));
        
        % 存储距离
        distances(m,1) = dist;
        m = m+1;
       
    end
end

max_dist = max(distances);


%计算所有交点的质心
mean_Intersections = mean(Intersections);

%计算交点质心到每个投影点的距离（对应着圆柱的半径）
for i=1:size(subpoints_proj_on_plane,1)
    Dist(i,1) = sqrt(sum((mean_Intersections(1,:) - subpoints_proj_on_plane(i,:)).^2));
end
R = mean(Dist);

%计算各半径之间的误差较
m=1;
for i=1:size(Dist,1)
    for j=i+1:size(Dist,1)
    E_ridus(m,1) = abs(Dist(i,1) - Dist(j,1));
    m=m+1;
end

max_E_ridus = max(E_ridus);


%根据不同的半径动态生成阈值
[ErrDisandRidus] = Calculate_intersection_distance_threshold(R);

if max_dist<ErrDisandRidus & max_E_ridus<ErrDisandRidus
    Bool_intersection_valid = 1;
end
end
