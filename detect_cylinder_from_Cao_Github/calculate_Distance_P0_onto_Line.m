function distance = calculate_Distance_P0_onto_Line(Pi, Ni, P0)
    % Pi: [x, y, z] 点Pi的坐标
    % Ni: [x, y, z] 方向向量Ni
    % P0: [x, y, z] 点P0的坐标
    % distance: 点P0到直线Li的距离

    % 计算向量PiP0
    PiP0 = P0 - Pi;

    % 计算向量Ni的模长
    Ni_norm = norm(Ni);

    % 计算向量PiP0与向量Ni的点积
    dot_product = dot(PiP0, Ni);

    % 计算向量PiP0与向量Ni的叉积
    cross_product = cross(PiP0, Ni);

    % 计算点P0到直线Li的距离
    distance = norm(cross_product) / Ni_norm;
end