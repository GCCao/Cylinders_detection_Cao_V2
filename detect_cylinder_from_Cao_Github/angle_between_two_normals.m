function [bool_valid_angle,theta_deg] = angle_between_two_normals(N1,N2,T_angle)

% 计算点积
dot_product = N1(1) * N2(1) + N1(2) * N2(2) + N1(3) * N2(3);

% 计算模长
norm_N1 = sqrt(N1(1)^2 + N1(2)^2 + N1(3)^2);
norm_N2 = sqrt(N2(1)^2 + N2(2)^2 + N2(3)^2);

% 计算夹角的余弦值
cos_theta = dot_product / (norm_N1 * norm_N2);

% 计算夹角的弧度值
theta_rad = acos(cos_theta);

% 将弧度转换为度数
theta_deg = radtodeg(theta_rad);

% 确保夹角在0到90度之间
if theta_deg  > 90
    theta_deg = 180 - theta_deg;
end

% 判断夹角是否大于T_angle度
bool_valid_angle = 1;
if abs(theta_deg - 90) > T_angle
    bool_valid_angle = 0;
end

end