%==========================================================================
% Calculate the distance from the point P0 to the axis P2-P1 of the Cylinder
% D = ||(P2-P1) x (P1-P0)|| / ||P2-P1||
% The distance from P0 to the cylinder is ||D - r||
% model是这个圆柱的参数
%==========================================================================
function dis = Calculate_the_distance_from_points_into_Cylinder(model, points)
p1p0 = [points(:,1)-model(1), points(:,2)-model(2), points(:,3)-model(3)];
p2p1 = model(4:6);
c = [p1p0(:,2)*p2p1(3) - p1p0(:,3)*p2p1(2), ...
    p1p0(:,3)*p2p1(1) - p1p0(:,1)*p2p1(3), ...
    p1p0(:,1)*p2p1(2) - p1p0(:,2)*p2p1(1)];
% p2p1 is a unit vector, so the denominator is not needed
D = sum(c.*c, 2);
dis = abs(sqrt(D) - model(7));

end