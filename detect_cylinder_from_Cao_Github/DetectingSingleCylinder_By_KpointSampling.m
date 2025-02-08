function [bestCylinderModel,predictCylinderModel,sigma,SeedPointIndexs,bestCylinderInliersIndex] = DetectingSingleCylinder_By_KpointSampling(currentClusterPoints,minClustersDistance,minSizeValidCluster,magnification_sigma,neighboringSize)
% 从当前族中检测圆柱体，并返回圆柱体模型，以及与该圆柱体关联的点的索引（这些点后续将被统一删除）

%加载圆柱体的误差特征向量，并进行学习
P = load("FeatureVectors.txt");
Mdl = KNN_FeatureVectors_Model(P);
sigma =[];
bestCylinderModel = {};
predictCylinderModel = [];
bestCylinderInliersIndex = [];

FeatureVector_CylindricalError = [];

currentClusterPointCloud = pointCloud(currentClusterPoints);
if isempty(currentClusterPointCloud.Normal)
    % Use 6 neighboring points to estimate a normal vector
    currentClusterPointsNormal = surfaceNormalImpl(currentClusterPointCloud, neighboringSize);
end

%%%%%%%%%%%%%%%%%%

% figureNum =15;
% figure(figureNum)
% pcshow(currentClusterPoints,[1,0,0],'MarkerSize',50); %显示点云数据
%%%%%%%%%%%%%%



K = 3;                    %任意选择K个点,根据他们法向两两叉乘是否同向，来初步判断他们是否位于圆柱体之上
maxIterations = 500000;   %最大迭代次数
iteration = 0;
Theda_T  = 5;             %点的法向量之间两两叉乘的结果是否同轴的判断阈值，单位：度
ErrDisandRidus = 5;       %如果半径之差小于ErrDisandRidus，并且交点之间的距离小于ErrDisandRidus，则认为是合格的圆柱
maxDistance  = 5;         %删除到圆柱体距离较远的点
maxAngleDiff = 5;         %删除法向量与圆柱轴线的夹角之差与90的差值较大的点，单位：度
minR = 2;                 %最小圆柱半径  
maxR = 500;               %最大圆柱半径
% minClustersDistance = 20; %聚类时用于分割点云族的阈值

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
            AngleDiff = Calculate_the_AngleDiff_between_the_normals_and_the_axis(cylinderParam, currentClusterPoints,currentClusterPointsNormal);

            %（5.5.3）找出所有的与种子点关联的内点Inliers,即距离和角度都初步满足要求的点
            AllassociatedPointsOfSeed = currentClusterPoints((dis < maxDistance & AngleDiff < maxAngleDiff),:);
            Indexs_AllassociatedPointsOfSeed_incurrent = find(dis < maxDistance & AngleDiff < maxAngleDiff); %关联点在currentClusterPoints的索引


            %%%%%%%%%%%
            if norm(currentClusterPoints(1,:)-currentClusterPoints(2,:))<0.001
                ee=0;
            end

            %（5.5.4）找到精准的圆柱。上面只是根据3点法得到了椭圆的参数，有一定的偏差，需要通过优化方法，找到精准的.
            % 此处bestCylinderInliersIndex是在currentClusterPoints中的索引
            [bestCylinderModel,bestCylinderInliersPoints,bestCylinderInliersIndex,sigma] = Fine_Cylinder_cluster(cylinderParam,AllassociatedPointsOfSeed,Indexs_AllassociatedPointsOfSeed_incurrent,minClustersDistance,minSizeValidCluster,SeedPointIndexs,currentClusterPoints,magnification_sigma);

            %(5.5.5）净化点云，更新圆柱的参数。有一些处于弯曲部分的点，但是满足距离和法向约束要求，被误以为是圆柱体上的点，应当被删除
            %                             这类点的特点是，在圆弧上的占比比较小，通常少于1/5
           
            if ~isempty(bestCylinderModel)
                if bestCylinderInliersIndex(1) == bestCylinderInliersIndex(2)
                    ee=0;
                end

                % %%%%%%%%%%----------------------------------------------------------------
                % % % % % % % %%%%%%%%%%
                % figure(20)
                % pcshow(bestCylinderInliersPoints,[0.5,0.5,0.5],'MarkerSize',50); %显示点云数据                  
                % % 绘制法向量
                % for i = 1:size(subpoints, 1)
                %     % 计算法向量在每个点的位置
                %     hold on
                %     plot3(subpoints(:,1), subpoints(:,2), subpoints(:,3), 'bo','MarkerSize',10);  % 'bo'表示蓝色圆点
                %     quiver3(subpoints(i,1), subpoints(i,2), subpoints(i,3), ...
                %         subnormal(i,1)*25, subnormal(i,2)*25, subnormal(i,3)*25, ...
                %         'MaxHeadSize', 5,'Color', 'red', 'LineWidth', 5); %MaxHeadSize表示箭头大小， 'r'表示红色箭头，'LineWidth'设置线宽，
                % end
                % axis equal
                % ee=0;
                % %%%%%%%%%---------------------------------------------------
            end


            if ~isempty(bestCylinderModel)
                A = bestCylinderModel.Parameters(1:3);
                B = bestCylinderModel.Parameters(4:6);

                AB_norm = norm(A-B);
                if AB_norm == 0
                    ee=0;
                end
            end


            if isempty(bestCylinderModel)
                bestCylinderModel = {};
                predictCylinderModel = [];
                SeedPointIndexs =[];
                bestCylinderInliersIndex = [];
                return;
            end

           

            %（5.5.8）如果检测到满足所有要求的圆柱
            ratio = bestCylinderModel.Radius / R;
            if ~isempty(bestCylinderModel) & bestCylinderModel.Radius>minR & bestCylinderModel.Radius <maxR &  ratio>0.8 & ratio<1.2 %& predictCylinderModel==1
               

              
                return;
            else

                % % % % % % % %%%%%%%%%%
                % figure(20)
                % 
                % pcshow(currentClusterPoints,[0.5,0.5,0.5],'MarkerSize',50); %显示点云数据
                %  hold on
                % pcshow(bestCylinderInliersPoints,[1,0.5,0.5],'MarkerSize',50); %显示点云数据
                % hold on
                % plot(bestCylinderModel);
                % Draw_arrows_VerifiedCylinders_Multi_scale_Samp(bestCylinderModel,1,10);
                % 
                % 
                % hold on
                % plot3(subpoints(:,1), subpoints(:,2), subpoints(:,3), 'bo','MarkerSize',10);  % 'bo'表示蓝色圆点
                % % 绘制法向量
                % for i = 1:size(subpoints, 1)
                %     % 计算法向量在每个点的位置
                %     figure(10)
                %     hold on
                %     quiver3(subpoints(i,1), subpoints(i,2), subpoints(i,3), ...
                %         subnormal(i,1)*50, subnormal(i,2)*50, subnormal(i,3)*50, ...
                %         'MaxHeadSize', 5,'Color', 'magenta', 'LineWidth', 5); %MaxHeadSize表示箭头大小， 'r'表示红色箭头，'LineWidth'设置线宽，
                % end
                % % % % % % 


                %%%%%%%%%%

                bestCylinderModel = {};
            end



        end


    end

end


end