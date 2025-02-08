function V_proj_on_plane = project_Vector_onto_plane(V, plane_normal)
%功能：计算一个方向向量在平面上的投影
%输入：方向向量V；平面法向量plane_normal，这两个向量不要求是单位向量。
%输出：投影得到向量,也不一定是单位向量


%计算向量V与平面法向量的点积：
dotprod  = dot(V,plane_normal);

%确定投影系数：使用点积和法向量模的平方，确定向量𝑉在平面法向量方向上的投影系数𝑘
k = dotprod/dot(plane_normal,plane_normal);

%计算投影向量：将投影系数𝑘乘以平面的法向量plane_normal,得到向量V在法向量方向上的投影向量 
V_proj_on_normal = k*plane_normal; 

%计算向量在平面上的投影：向量𝑉在平面上的投影V_proj_on_plane是原始向量减去其在法向量方向上的投影：
V_proj_on_plane = V - V_proj_on_normal;

end