function P_prj = project_Point_onto_plane(P, a,b,c,d)
%功能：计算一个空间点在平面上的投影点
%输入：空间点坐标P =（x0,y0,z0）；平面方程的系数ax+bx+cz+d=0。
%输出：投影得空间点坐标

x0 = P(1); y0 = P(2); z0 = P(3);


%过点(x0,y0,z0)，且与平面垂直的直线方程为
%（x-x0）/a =  (y-y0)/b = (z-z0)/c  = t

%点到平面的投影就是上述直线与平面的交点，注意到直线的参数方程为
% x = at+x0; y= bt+y0; z = ct+z0
% 带入到平面方程，可以求解处t
t = -(a*x0 + b*y0 +c*z0 +d) / (a^2 + b^2 +c^2);

%于是，交点为
x = a*t +x0; 
y = b*t +y0;
z = c*t +z0;

P_prj = [x y z];

end