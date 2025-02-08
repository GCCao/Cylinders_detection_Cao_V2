
function Index_bestCylinderInliers_inValidClusterPoints = Find_points_3sigma_frome_cylindersurface(sigma,bestCylinderModel,ValidClusterPoints)
%找到距离圆柱体表面3倍sigma的点

model = [bestCylinderModel.Center bestCylinderModel.Orientation/norm(bestCylinderModel.Orientation) bestCylinderModel.Radius];

% Calculate the distance from the point P0 to the axis P2-P1 of the Cylinder
% D = ||(P2-P1) x (P1-P0)|| / ||P2-P1||
p1p0 = [ValidClusterPoints(:,1)-model(1), ValidClusterPoints(:,2)-model(2), ValidClusterPoints(:,3)-model(3)];
p2p1 = model(4:6);
c = [p1p0(:,2)*p2p1(3) - p1p0(:,3)*p2p1(2), ...
    p1p0(:,3)*p2p1(1) - p1p0(:,1)*p2p1(3), ...
    p1p0(:,1)*p2p1(2) - p1p0(:,2)*p2p1(1)];
% p2p1 is a unit vector, so the denominator is not needed
D = sum(c.*c, 2);


%如果点位于膨胀圆柱体内部,并且聚类的标签与种子点属于同一个标签，则被删除
Index_bestCylinderInliers_inValidClusterPoints = find( abs(sqrt(D)- model(7))< 1*sigma ) ; %关联点在remainingPointCloud的索引
end
