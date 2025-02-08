function indices = find_indices_kdtree(P, B)
% 该函数的目的是找到B中所有点在P中的索引

% P 是一个 N*3 的点云
% B 是 P 的一个子集，尺寸为 M*3
% 返回 B 中所有点在 P 中的索引

[M, ~] = size(B);
indices = zeros(M, 1);

kdtree = createns(P, 'nsmethod', 'kdtree', 'distance', 'euclidean');
for i = 1:M
    [~, min_index] = knnsearch(kdtree, B(i, :), 'k', 1, 'distance', 'euclidean');
    indices(i) = min_index;
end

end