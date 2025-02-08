function [Featurevectors_context] = Construct_featurevectors_basedon_context(detectedCylinders)
%根据上下文，构建每个圆柱的特征向量
%思路：计算每个圆柱的最近K邻域，计算圆柱轴线与每个邻域轴线的夹角，得到Theda1,...,ThedaK

K = 3; %圆柱的邻域个数

if size(detectedCylinders,2)<(K+1)
    % warning("圆柱数量不足，无法生成上下文特征矢量")
    error('圆柱数量不足，无法生成上下文特征矢量')
end

CylinderCenters =[];
CylinderStartPoints =[];
CylinderEndPoints =[];
for i=1:size(detectedCylinders,2)
    bestCylinderModel = detectedCylinders{i};
    CylinderCenters = [CylinderCenters;bestCylinderModel.Center];
    CylinderStartPoints = [CylinderStartPoints;bestCylinderModel.Parameters(1:3)];
    CylinderEndPoints = [CylinderEndPoints;bestCylinderModel.Parameters(1:3)];    
end

%计算每个圆柱体K最近邻圆柱体索引
indices = [];
for i=1:size(detectedCylinders,2)

    A = [CylinderCenters(i,:);CylinderStartPoints(i,:);CylinderEndPoints(i,:)];

    dist =[];
    for j = 1: size(detectedCylinders,2)
        if i==j
            dist(j,1) = inf;
        else
            B = [CylinderCenters(j,:);CylinderStartPoints(j,:);CylinderEndPoints(j,:)];
            dist(j,1) = calculateHausdorffDistance(A, B);
        end
    end

    %找到最小的K个元素，并返回索引   
    [ind] = findKSmallestIndices(dist, K);

    indices = [indices;ind'];
end

%计算每个圆柱轴线与邻域圆柱轴线的夹角
Featurevectors_context = [];
for i=1:size(detectedCylinders,2)
    ind = indices(i,:);
    bestCylinderModel = detectedCylinders{i};
    N1 = bestCylinderModel.Orientation;
    angelVectors = [];
    for j=1:size(ind,2)
        Model = detectedCylinders{ind(j)};
        N2 = Model.Orientation;
        angleInDegrees = calculateAngleBetweenVectors(N1, N2);  

        if angleInDegrees>90
            angleInDegrees = 180-angleInDegrees;
        end

        angelVectors =[angelVectors angleInDegrees];
    end

    ratio_HvsDiam =  bestCylinderModel.Height/(2*bestCylinderModel.Radius);
    Featurevectors_context = [Featurevectors_context;angelVectors ratio_HvsDiam];
    
    
end



end