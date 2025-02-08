function [cylinderModel,Index_detectedPoints_inremain] = DetectingSingleCylinder_By_KpointSampling(currentClusterPoints,Index_currentClusterPoints_inremain,remainingPointCloud)
% 从当前族中检测圆柱体，并返回圆柱体模型，以及与该圆柱体关联的点的索引（这些点后续将被统一删除）

currentClusterPointCloud = pointCloud(currentClusterPoints);
currentClusterPointsNormal = currentClusterPointCloud.Normal;

K = 3;                    %任意选择K个点,根据他们法向两两叉乘是否同向，来初步判断他们是否位于圆柱体之上
maxIterations = 100000;   %最大迭代次数
iteration = 0;
Theda_T  = 5;             %点的法向量之间两两叉乘的结果是否同轴的判断阈值，单位：度
ErrDisandRidus = 5;       %如果半径之差小于ErrDisandRidus，并且交点之间的距离小于ErrDisandRidus，则认为是合格的圆柱
maxDistance  = 5;         %删除到圆柱体距离较远的点
maxAngleDiff = 5;         %删除法向量与圆柱轴线的夹角之差与90的差值较大的点，单位：度
minR = 3;                 %最小圆柱半径  
maxR = 250;               %最大圆柱半径
minClustersDistance = 20; %聚类时用于分割点云族的阈值

while iteration < maxIterations
    
    iteration = iteration + 1;    

    %（1） 获取currentClusterPoints点云中的点的数量
    numPoints = size(currentClusterPoints, 1);

    %（2） 在currentClusterPoints中随机选择K个不同的索引，用作采样
    SeedPointIndexs = randperm(numPoints, K);

    %（3） 根据索引获取采样点和法向量
    subpoints = currentClusterPoints(SeedPointIndexs, :);
    subnormal = currentClusterPointsNormal(SeedPointIndexs, :);
    
    %（4） 计算圆柱体上K个点的法向量之间两两叉乘的结果是否同轴.只要任意两个叉乘后的矢量夹角大于Theda_T，则不同轴，即认为这K个点不位于同一个圆柱体
    [bool_Are_collinear,Mean_vector] = Are_collinear_cross_products(subpoints,subnormal,K,Theda_T);


    %（5） 如果共线，说明是潜在的圆柱体，继续验证真伪
    if bool_Are_collinear

        %（5.1）---接下来，构建横截面平面方程。把选中的K个点投影到一个平面，要求该平面的法向与圆柱轴向同向，并让他经过其中一点
        %---------------先计算该平面的参数ax+bx+cz+d=0
        %---------------已知平面方程的系数（a,b,c），还有平面上一点P(x0,y0,z0)，求d
        a = Mean_vector(1);  b = Mean_vector(2); c = Mean_vector(3);
        x0 = subpoints(1,1); y0 = subpoints(1,2); z0 = subpoints(1,3);
        d = -(a*x0+b*y0+c*z0);

        %（5.2）---将候选的K个点以及对应的法向量，投影到该兴趣平面上，得到投影点和投影后的法向量
        [subpoints_proj_on_plane,subnormal_proj_on_plane] = project_Points_and_Vectors_onto_plane(subpoints,subnormal,a,b,c,d);

        %（5.3）---同一个平面上多条直线的交点
        Intersections = Lines_Intersection3D(subpoints_proj_on_plane,subnormal_proj_on_plane);

        %（5.4）---计算交点的位置关系，并判断他们是否处于圆柱的轴向上
        %如果半径之差小于ErrDisandRidus，并且交点之间的距离小于ErrDisandRidus，则认为是合格的圆柱
        [Bool_intersection_valid,max_dist,max_E_ridus,mean_Intersections,R]  = Is_the_intersections_located_on_the_axis_of_the_cylinder(Intersections,subpoints_proj_on_plane,ErrDisandRidus);


        %（5.5）---如果交点合理，半径也在规定范围，则继续验证
        if (Bool_intersection_valid & R>minR & R<maxR )

            %（5.5.1）构建圆柱体的初始参数,
            cylinderParam = [mean_Intersections Mean_vector R];

            %（5.5.2） 删除对候选圆柱体支撑性不足的点，即距离圆柱较远，或者法向与轴线夹角之差与90度相差较大的点
            dis = Calculate_the_distance_from_points_into_Cylinder(cylinderParam, currentClusterPoints);
            AngleDiff = Calculate_the_AngleDiff_between_the_normals_and_the_axis(model, currentClusterPoints,currentClusterPointsNormal);

            %（5.5.3）找出所有的与种子点关联的内点Inliers,即距离和角度都初步满足要求的点
            AllassociatedPointsOfSeed = currentClusterPoints((dis < maxDistance & AngleDiff < maxAngleDiff),:);
            Indexs_AllassociatedPointsOfSeed_incurrentClusterPoints = find(dis < maxDistance & AngleDiff < maxAngleDiff); %关联点在currentClusterPoints的索引

            %（5.5.4）找到精准的圆柱。上面只是根据3点法得到了椭圆的参数，有一定的偏差，需要通过优化方法，找到精准的
            [bestCylinderModel,bestCylinderInliersPoints,bestCylinderInliersIndex,sigma] = Fine_Cylinder_cluster(cylinderParam,AllassociatedPointsOfSeed,Indexs_AllassociatedPointsOfSeed_incurrentClusterPoints,minClustersDistance,SeedPointIndexs,currentClusterPoints);



        end


    end

end


end