function directions = sphericalFibonacciSampling(n)
    % n: Number of sampling points
    % directions: n-by-3 matrix, each row is a direction vector on the unit hemisphere

    golden_ratio = (1 + sqrt(5)) / 2;
    i = (0:n-1)';
    
    % Generate spherical Fibonacci lattice points
    theta = 2 * pi * i / golden_ratio;
    phi = acos(1 - 2 * (i + 0.5) / n);
    
    % Convert spherical coordinates to Cartesian coordinates
    x = cos(theta) .* sin(phi);
    y = sin(theta) .* sin(phi);
    z = cos(phi);
    
    % Keep only the points on the upper hemisphere (z >= 0)
    directions = [x, y, z];
    
    % Filter out the directions where z < 0 to stay in the upper hemisphere
    directions = directions(z >= 0, :);
end

