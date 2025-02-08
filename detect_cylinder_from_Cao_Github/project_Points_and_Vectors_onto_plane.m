function [subpoints_proj_on_plane,subnormal_proj_on_plane] = project_Points_and_Vectors_onto_plane(subpoints,subnormal,a,b,c,d)
%将候选的K个点以及对应的法向量，投影到该兴趣平面上

subpoints_proj_on_plane = [];
subnormal_proj_on_plane = [];

for i=1:size(subpoints,1)

    %计算一个空间点在平面上的投影点
    subpoints_proj_on_plane(i,:) = project_Point_onto_plane(subpoints(i,:), a,b,c,d);

    %计算一个方向向量在平面上的投影
    subnormal_proj_on_plane(i,:) = project_Vector_onto_plane(subnormal(i,:), [a,b,c]);

end

end