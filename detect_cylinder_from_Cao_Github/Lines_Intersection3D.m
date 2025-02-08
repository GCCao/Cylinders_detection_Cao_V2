function Intersections = Lines_Intersection3D(Points,Normal)
%同一个平面上多条直线的交点
%Points: n个点
%Normal：过n个直线对应的方向向量


Intersections =[];
k = 1;
for i=1:size(Points,1)
    for j = i+1 : size(Points,1)

        Intersection = Line_Line_Intersection3D(Points(i,:),Normal(i,:),Points(j,:),Normal(j,:));
        Intersections(k,:) = Intersection;
        k = k+1;
    end 
end

end