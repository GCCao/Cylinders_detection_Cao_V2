function [startp,endp] = params2endpoints(inlier,vector,point)
x = inlier(:,1);
y = inlier(:,2);
% find 2D (x-y) boundary
% figure;
% plot(x,y,'.')
k = boundary(x,y);
% hold on;
% plot(x(k),y(k));
% axis equal

% find line equation y = a*x +b
a = vector(2)/vector(1);
b = point(2) - a*point(1);
% find start point and endpoint of the line
p_val = [a, b];
x_val = [min(x)-1; max(x)+1];
y_val = polyval(p_val, x_val);
% intersection find 2D start point and endpoint of the tube 
xbox = x(k)';
ybox = y(k)';
% mapshow(xbox,ybox,'DisplayType','polygon','LineStyle','none')
% mapshow(x_val',y_val','Marker','+')
[xi,yi] = polyxpoly(x_val,y_val,xbox,ybox,'unique');
% mapshow(xi,yi,'DisplayType','point','Marker','o')
% find Z coordinates
zi = ((xi-point(1))/vector(1))*vector(3)+point(3); %((yi-V(5))/V(2))*V(3)+V(6)

startp = [xi(1), yi(1), zi(1)];
endp = [xi(2), yi(2), zi(2)];
end