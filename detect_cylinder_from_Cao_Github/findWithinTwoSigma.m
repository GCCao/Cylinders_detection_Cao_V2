function indices = findWithinTwoSigma(counts)
    %找到位于2倍sigma之内的元素
    
    
    % 计算平均值
    mean_val = mean(counts);
    
    % 计算标准差
    std_dev = std(counts);
    
    % 计算±2个标准差的范围
    lower_bound = mean_val - 2 * std_dev;
    upper_bound = mean_val + 2 * std_dev;
    
    % 找到落在这个范围内的元素的索引
    indices = find(counts >= lower_bound & counts <= upper_bound);
end