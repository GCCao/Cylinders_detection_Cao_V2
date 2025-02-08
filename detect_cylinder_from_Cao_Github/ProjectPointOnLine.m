function p_result = ProjectPointOnLine(X0,normal,p)
%计算p在直线上的投影，适用于2D或者3D
%X0是直线上一点的坐标，normal是直线的方向向量
%p是直线外一点

a = X0;
b = X0 + normal;

%假设a和b是直线上的两点，p是不在直线上的另一个点
%则p在ab上的投影为p1 =  a + dot(ap,ab)/dot(ab,ab) * ab
ap = p-a;
ab = b-a;
p_result = a + dot(ap,ab)/dot(ab,ab) * ab;


end