function projected_line = project_line_to_plane(line_point, line_dir, plane_point, plane_normal)
    % line_point: 直线L上的一点 P0
    % line_dir: 直线L的方向向量 d
    % plane_point: 平面P上的一点 P1
    % plane_normal: 平面P的法向量 n
    
    % 直线的参数方程代入平面方程
    t_eqn = @(t) plane_normal * (line_point + t * line_dir - plane_point);
    
    % 求t值，这里假设有唯一解
    t_value = fzero(t_eqn, 0); % fzero函数找到使得t_eqn为0的t值
    
    % 计算投影点
    projected_point = line_point + t_value * line_dir;
    
    % 投影直线的参数方程为：
    % r(t) = projected_point + t * line_dir
    % 这里我们只需要返回投影点和方向向量
    projected_line = struct('point', projected_point, 'direction', line_dir);
end

