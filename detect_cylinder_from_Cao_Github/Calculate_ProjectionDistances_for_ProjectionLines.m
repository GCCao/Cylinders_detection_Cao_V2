function [meanDis, sigmaDis] = Calculate_ProjectionDistances_for_ProjectionLines(bestCylinderModel,bestCylinderInliersIndex,remainingPointCloud,remainingPointCloudNormal)
 %计算合法圆柱点的法向量在横截面上的投影线，并计算中心点到每条投影线的距离,并计算平均距离和方差

 %计算圆柱的横截面的平面方程系数a b c d
 %先计算该平面的参数ax+bx+cz+d=0
 %已知平面方程的系数（a,b,c），还有平面上一点P(x0,y0,z0)，求d
 a = bestCylinderModel.Orientation(1);  b = bestCylinderModel.Orientation(2); c = bestCylinderModel.Orientation(3);
 x0 = bestCylinderModel.Parameters(1); y0 = bestCylinderModel.Parameters(2); z0 = bestCylinderModel.Parameters(3);
 d = -(a*x0+b*y0+c*z0);

 %将候选的圆柱点以及对应的法向量，投影到该兴趣平面上，subpoints_proj_on_plane是投影的点，subnormal_proj_on_plane是对应点在平面上的法向量投影
 subpoints = remainingPointCloud(bestCylinderInliersIndex,:);
 subnormal = remainingPointCloudNormal(bestCylinderInliersIndex,:);
 [subpoints_proj_on_plane,subnormal_proj_on_plane] = project_Points_and_Vectors_onto_plane(subpoints,subnormal,a,b,c,d);


 %计算横截面中心点P0到每条投影线的距离
 P0 = [x0,y0,z0];
 distance = [];
 for i=1:size(subpoints_proj_on_plane,1)
     Pi = subpoints_proj_on_plane(i,:);
     Ni = subnormal_proj_on_plane(i,:);
     distance(i,1) = calculate_Distance_P0_onto_Line(Pi, Ni, P0);
 end
 meanDis  = mean(distance);
 sigmaDis = var(distance);

end