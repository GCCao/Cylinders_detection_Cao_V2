function Intersection = Line_Line_Intersection3D(P1,normal1,P2,normal2)
%假设有2条线，并且假设他们是共面的，求他们交点
%输入：直线L1：P1,normal1；直线L2：P2,normal2

Intersection = [];

%L1的参数形式方程：
% x = P1(1) + normal1(1)*t
% y = P1(2) + normal1(2)*t
% z = P1(3) + normal1(3)*t


%L2的参数形式方程：
% x = P2(1) + normal2(1)*s
% y = P2(2) + normal2(2)*s
% z = P2(3) + normal2(3)*s

%根据相交，则以上几个方程相等，简化之后，可以得到
% AX = Y
%其中A为
A = [normal1; -normal2]';
%X = [t s]'
Y = [P2-P1]';

X = (A'*A)^(-1)*A'*Y;

t = X(1);
s = X(2);

Intersection(1,1) =  P1(1) + normal1(1)*t;
Intersection(1,2) =  P1(2) + normal1(2)*t;
Intersection(1,3) =  P1(3) + normal1(3)*t;




end
