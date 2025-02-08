function P_noisy = addNoiseToPointCloud(P,std_percentages)  
    % 输入：  
    %   P: 原始点云数据 (N x 3)  
    % 输出：  
    %   P_noisy: 包含不同噪声水平的点云数据的单元数组  

    % 计算边界框的最小值和最大值  
    min_vals = min(P, [], 1);  
    max_vals = max(P, [], 1);  

    % 计算边界框每个维度的尺寸  
    bbox_dims = max_vals - min_vals;  

    % 确定立方体边界框的尺寸（取所有维度中的最大值）  
    cube_size = max(bbox_dims);  

    % 定义噪声标准差（相对于立方体边界框尺寸的百分比）  
    %std_percentages = [0.001, 0.00125, 0.0025];  % 0.1%、0.125%、0.25%  
    std_devs = cube_size * std_percentages;      % 实际标准差值  

    % 初始化存储扰动后点云的单元数组  
    P_noisy = cell(length(std_devs), 1);  

    % 对每个噪声水平，添加高斯噪声  
    for i = 1:length(std_devs)  
        noise_std = std_devs(i);  
        noise = randn(size(P)) * noise_std;  
        P_noisy{i} = P + noise;  
    end  
end