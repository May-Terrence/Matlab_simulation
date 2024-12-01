function collision_flag = collision_check(map,x_nearest,x_new)
collision_flag = 1; % 默认不碰撞
% 判断采样点是否位于障碍区
is_in = 0;
for i = 1:length(map.obs_boundary)
    obs_boundary_i = map.obs_boundary{1,i};
    in = inpolygon(x_new(1),x_new(2),obs_boundary_i{1,1}(:,1),obs_boundary_i{1,1}(:,2));
    if in == 1
        is_in = 1;
        break
    end
end
for i = 1:length(map.obs_polygon)
    obs_polygon_i = map.obs_polygon{1,i};
    in = inpolygon(x_new(1),x_new(2),obs_polygon_i{1,1}(:,1),obs_polygon_i{1,1}(:,2));
    if in == 1
        is_in = 1;
        break
    end
end
for i = 1:size(map.obs_circle,1)
    t = linspace(0, 2*pi);
    x = cos(t)*map.obs_circle(i,3)+map.obs_circle(i,1);
    y = sin(t)*map.obs_circle(i,3)+map.obs_circle(i,2);
    in = inpolygon(x_new(1),x_new(2),x,y);
    if in == 1
        is_in = 1;
        break
    end
end
if is_in == 0
    is_collision = false;
    % 碰撞检测

    % 多边形障碍物检测
    polygon_num = length(map.obs_polygon);
    for k = 1:polygon_num
        vertex_num = size(map.obs_polygon{1,k}{1,1},1);
        for m = 1:vertex_num
            if m == vertex_num
                A = map.obs_polygon{1,k}{1,1}(m,:);
                B = map.obs_polygon{1,k}{1,1}(1,:);
            else
                A = map.obs_polygon{1,k}{1,1}(m,:);
                B = map.obs_polygon{1,k}{1,1}(m+1,:);
            end
            is_intersect = intersect_check(A,B,x_nearest,x_new);
            if is_intersect == 1
                is_collision = true;
            end
        end
    end

    % 多边形障碍物检测
    polygon_num = length(map.obs_boundary);
    for k = 1:polygon_num
        vertex_num = size(map.obs_boundary{1,k}{1,1},1);
        for m = 1:vertex_num
            if m == vertex_num
                A = map.obs_boundary{1,k}{1,1}(m,:);
                B = map.obs_boundary{1,k}{1,1}(1,:);
            else
                A = map.obs_boundary{1,k}{1,1}(m,:);
                B = map.obs_boundary{1,k}{1,1}(m+1,:);
            end
            is_intersect = intersect_check(A,B,x_nearest,x_new);
            if is_intersect == 1
                is_collision = true;
            end
        end
    end
    
    % 圆形障碍物检测
    for k = 1:size(map.obs_circle,1)
        lambda = dot(map.obs_circle(k,1:2)-x_nearest,x_new-x_nearest) / (norm(x_new-x_nearest)*norm(x_new-x_nearest));
        if lambda < 1 && lambda > 0
            g = x_nearest + lambda*(x_new-x_nearest);
            d = norm(g-map.obs_circle(k,1:2));
            if d < map.obs_circle(k,3)
                is_collision = true;
            end
        end
    end
    
    if is_collision == true
        collision_flag = 0;
    end
else
    collision_flag = 0;
end
