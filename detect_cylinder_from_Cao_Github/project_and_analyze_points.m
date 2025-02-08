function std_val = project_and_analyze_points(P, n1, r, center)  
%此函数将点云 P 投影到平面 F 上，将平面分成100个均等的扇区，计算每个扇区中的点数，去除点数少于5的扇区，然后对点数进行标准化并计算标准差。
   % P: N*3点云  
    % n1: 1*3的向量，为轴向量  
    % r: 圆柱的半径（未使用）  
    % center: 1*3的坐标，圆柱轴线上一点，平面F经过  

    % ensure n1 is a unit vector  
    n1 = n1 / norm(n1);  

    % Compute the projection of each point on the plane  
    % Projection formula: P_proj = P - dot(P-C, n1).*n1  
    Pc = bsxfun(@minus, P, center); % Translate points by center  
    dot_products = dot(Pc, repmat(n1, size(P, 1), 1), 2);  
    P_proj = Pc - dot_products .* repmat(n1, size(P, 1), 1);  
    
    % Get 2D coordinates on the plane F  
    % Choose an arbitrary orthogonal basis for the plane  
    [U, ~, ~] = svd(null(n1(:)')); % U will have 2 columns, a 2D basis in the plane  
    P2D = P_proj * U; % Project to 2D on the plane  
    
    % Convert to polar coordinates (angle and radius) centered at origin  
    theta = atan2(P2D(:,2), P2D(:,1));  
    
    % Scale angles to [0, 2*pi]  
    theta(theta < 0) = theta(theta < 0) + 2*pi;  
    
    % Divide the circle into 20 bins  
    num_bins = 20;  
    bin_edges = linspace(0, 2*pi, num_bins + 1);  
    
    % 修改这部分，确保bin_idx为整数  
    bin_idx = zeros(size(theta));  
    for i = 1:length(theta)  
        % 找到theta位于哪个bin中  
        bin_idx(i) = find(bin_edges(1:end-1) <= theta(i) & bin_edges(2:end) > theta(i), 1);  
    end  
    
    % 确保所有点都被分配到bin中  
    bin_idx(bin_idx == 0) = num_bins;  

    % Count points in each bin  
    point_counts = accumarray(bin_idx, 1, [num_bins 1]);  
    
    % Remove bins with fewer than 5 points  
    point_counts(point_counts < 5) = 0;  
    
    % Find max number of points in a bin  
    maxnum = max(point_counts);  
    
    % Normalize the point counts  
    normalized_counts = point_counts / maxnum;  
    
    % Compute standard deviation, ignoring zero counts  
    std_val = std(normalized_counts(normalized_counts > 0));  
    disp(['Standard deviation of normalized counts: ', num2str(std_val)]);
end