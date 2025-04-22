% 四类B样条曲线的画图
% clc
% clear
close all
global path_opt path_Uniform_BSpline path_len
%% 数据定义
k = 4;                                    % k阶、k-1次B样条
% % P = [0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4;
% %     0, 0.5, 1, 0.5, 0, 0.5, 1, 0.5, 0];            

% P = [13, 10, 11, 13.5,   15, 16.8;
%       5,  8, 11,   12, 11.5,  11];
% P = [0, 1, 2, 3, 4;
%     0, 1, 1, 2, 4]; 
% P = [0, 2, 4, 6, 8, 10;
%     0, 2, 4, 6, 8, 10]; 
% P = P*10;
P = path_opt';
% P = [0, 0.5,1, 2, 3, 4;
%     0, 0.5,1, 1, 2, 4]; 
% P = [P(:,1), P(:,:), P(:,end)];
P = [P(:,1), P(:,1),P(:,:), P(:,end),P(:,end)];
% P = [P(:,1), P(:,:), P(:,end)];
% P = P1;
n = size(P,2) - 1; 
% P = path_opt';
P1 = diff(P, 1, 2);

n1 = size(P1,2) - 1;                          % n是控制点个数，从0开始计数
sample = (n-k+2)*100;
% nodeVector = nodeVector_tmp;
%% 生成B样条曲线
Bik = zeros(n+1, 1);
for i = 1
    nodeVector = getNodeVector(n, k, i);
    vec_u = linspace(nodeVector(k), nodeVector(n+2), sample);
    if i == 1   % 均匀B样条
        path_Uniform_BSpline = [];
        % vec_u = linspace((k-1)/(n+k), 1-(k-1)/(n+k), sample);
        % vec_u = linspace(nodeVector(k), nodeVector(n+2), sample);
        % for u = (k-1)/(n+k) : 0.005 : 1-(k-1)/(n+k) 
        for it = 1 : 1 : size(vec_u,2)
            u = vec_u(it);
            for j = 0 : 1 : n
                Bik(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector);
            end
            p_u = [P * Bik;u-(k-1)/(n+k)];
            path_Uniform_BSpline(end+1,:) = p_u;
        end
    elseif i == 2 % 准均匀B样条
        path_Quasi_Uniform_BSpline = [];
        for u = 0 : 0.005 : 1-0.005
            for j = 0 : 1 : n
                Bik(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector);
            end
            p_u = [P * Bik;u];
            path_Quasi_Uniform_BSpline(end+1,:) = p_u;
        end
        % for it = 1 : 1 : size(vec_u,2)
        %     u = vec_u(it);
        %     for j = 0 : 1 : n
        %         Bik(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector);
        %     end
        %     p_u = [P * Bik;u];
        %     path_Quasi_Uniform_BSpline(end+1,:) = p_u;
        % end
    end
end

%% 生成B样条曲线
k = k-1;
alpha = zeros(sample,1);
Bik1 = zeros(n1+1, 1);
v_max = 3;
for i = 1
    nodeVector1 = getNodeVector(n1, k, i);
    for j = 0 : 1 : n1
        P1(:,j+1) = P1(:,j+1)*k/(nodeVector(j+2+k)-nodeVector(j+2));
    end
    P1_tmp = P1;

    if i == 1   % 均匀B样条
        path_Uniform_BSpline1 = [];
        path_Uniform_BSpline1_old = [];
        nodeVector_tmp = nodeVector;
        % u = (k-1)/(n1+k);
        vec_u1 = linspace((k-1)/(n1+k), 1-(k-1)/(n1+k), sample);
        it = 1;
        while it <= size(vec_u1,2) 
            u = vec_u1(it);
            for j = 0 : 1 : n1
                Bik1(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector1);
            end
            path_Uniform_BSpline1_old(end+1,:) = P1 * Bik1;
            it = it + 1;
        end
        P1_tmp = node_limit(P1,v_max);
        % P1_tmp = P1_tmp/2;
        % P1_tmp(:,3) = P1_tmp(:,3)/2;
        it = 1;
        while it <= size(vec_u1,2) 
            u = vec_u1(it);
            for j = 0 : 1 : n1
                Bik1(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector1);
            end
            % if norm(P1_tmp * Bik1)/v_max > 1.0
            %     idx = getIdx(u,nodeVector);
            %     alpha(it) = norm(P1_tmp * Bik1)/v_max;
            %     for a = idx-k+1 : 1 : idx
            %         P1_tmp(:,a) = P1_tmp(:,a) / alpha(it);
            %     end
            %     % u = u - 0.005; 
            %     it = it - 1;
            % else
                p_u1 = [P1_tmp * Bik1;(u-(k-1)/(n1+k))];
                path_Uniform_BSpline1(end+1,:) = p_u1;
            % end
            it = it + 1;
            % u = u + 0.005;
        end 
        time = adjust_time(path_Uniform_BSpline(:,3), sqrt(sum(path_Uniform_BSpline1_old(:,1:2).^2, 2)), sqrt(sum(path_Uniform_BSpline1(:,1:2).^2, 2)));

        for a = 1 : 1 : size(P1_tmp,2)
            if norm(P1(:,a))~= 0 && norm(P1(:,a)) / norm(P1_tmp(:,a)) > 1
                beta = norm(P1(:,a)) / norm(P1_tmp(:,a));
                for p = a+2 : 1 : a+k+1
                    % nodeVector_tmp(a:a+k) = nodeVector_tmp(a:a+k) + nodeVector(a:a+k)*(alpha-1);
                    nodeVector_tmp(p) = nodeVector_tmp(p-1) + (nodeVector(p)-nodeVector(p-1))*(beta);
                end
                nodeVector_tmp(a+k+2:end) = nodeVector_tmp(a+k+2:end) + nodeVector_tmp(p) - nodeVector(p);
            end
        end
        % 
        % time = [];
        % for a = k : 1 : n-1%k-1/n+1
        %   time = [time, linspace(nodeVector_tmp(a+1),nodeVector_tmp(a+2), 11)];
        %   time(end) = [];
        % end
        % a = n;
        % time = [time, linspace(nodeVector_tmp(a+1),nodeVector_tmp(a+2), 10)];
        % time = time - time(1);

    elseif i == 2 % 准均匀B样条
        path_Quasi_Uniform_BSpline1 = [];
            u = 0;
            while u < 1
                for j = 0 : 1 : n1
                    Bik1(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector1);
                end
                p_u1 = [P1 * Bik1;u];
                path_Quasi_Uniform_BSpline1(end+1,:) = p_u1;
                u = u + 0.005;
            end
    end
end

P2 = diff(P1_tmp, 1, 2);
n2 = size(P2,2) - 1; 
a_max = 2;
%% 生成B样条曲线
k = k-1;
nodeVector = nodeVector_tmp;
Bik2 = zeros(n2+1, 1);
for i = 1
    nodeVector2 = getNodeVector(n2, k, i);
    for j = 0 : 1 : n2
        P2(:,j+1) = P2(:,j+1)*k/(nodeVector(j+3+k)-nodeVector(j+3));
    end
    if i == 1   % 均匀B样条
        % P2 = P2 / time(end);
        % P2 = P2 / (time(end)/path_Uniform_BSpline(end,3));

        path_Uniform_BSpline2 = [];
        path_Uniform_BSpline2_old = [];
        vec_u2 = linspace((k-1)/(n2+k), 1-(k-1)/(n2+k), sample);
        it = 1;
        while it <= size(vec_u2,2) 
            u = vec_u2(it);
            for j = 0 : 1 : n2
                Bik2(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector2);
            end
            path_Uniform_BSpline2_old(end+1,:) = P2 * Bik2;
            it = it + 1;
        end
        P2_tmp = node_limit(P2,a_max);
        % P2_tmp = P2;
        it = 1;
        while it <= size(vec_u2,2) 
            u = vec_u2(it);
            for j = 0 : 1 : n2
                Bik2(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector2);
            end
                p_u2 = P2_tmp * Bik2;
                path_Uniform_BSpline2(end+1,:) = p_u2;
            it = it + 1;
        end
        time2 = adjust_acc_time(time, sqrt(sum(path_Uniform_BSpline2_old(:,1:2).^2, 2)), sqrt(sum(path_Uniform_BSpline2(:,1:2).^2, 2)));
        % time2 = adjust_time(time, sqrt(sum(path_Uniform_BSpline2_old(:,1:2).^2, 2)), sqrt(sum(path_Uniform_BSpline2(:,1:2).^2, 2)));
        % time2 = path_Uniform_BSpline1(:,3);
        % time2 = time;
        % time2 = time / sqrt(5);
        time = time2;
    elseif i == 2 % 准均匀B样条
        path_Quasi_Uniform_BSpline2 = [];
        % nodeVector2_1 = nodeVector2*alpha;
        %     for j = 0 : 1 : n2
        %         P2(:,j+1) = P2(:,j+1)*k/(nodeVector2_1(j+1+k)-nodeVector2_1(j+1));
        %     end
            for u = 0 : 0.005 : 1-0.005
                for j = 0 : 1 : n2
                    Bik2(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector2);
                end
                p_u2 = [P2 * Bik2;u];
                path_Quasi_Uniform_BSpline2(end+1,:) = p_u2;
            end
    end
end

%% 画图
% 画均匀B样条曲线
figure
hold on
grid on
plot(P(1,:), P(2,:),'LineWidth', 3,  'Color', 'b');
plot(path_Uniform_BSpline(:,1),path_Uniform_BSpline(:,2),'LineWidth', 3,  'Color', 'c')

[Px, Time] = time_adjust(path_Uniform_BSpline(:,1),time);
% plot(time,path_Uniform_BSpline(:,1),'LineWidth', 3,  'Color', 'r')
plot(Time,Px,'LineWidth', 3,  'Color', 'r')

[Py, Time] = time_adjust(path_Uniform_BSpline(:,2),time);
% plot(time,path_Uniform_BSpline(:,2),'LineWidth', 3,  'Color', 'g')
plot(Time,Py,'LineWidth', 3,  'Color', 'g')

scatter(P(1,:), P(2,:), 40,'MarkerEdgeColor','g',...
    'MarkerFaceColor','g');

figure
hold on
grid on
[Vx1, Time] = time_adjust(path_Uniform_BSpline1(:,1),time);
[Vy1, Time] = time_adjust(path_Uniform_BSpline1(:,2),time);

plot(time,path_Uniform_BSpline1(:,1),'LineWidth', 3,  'Color', 'r')
plot(time,path_Uniform_BSpline1(:,2),'LineWidth', 3,  'Color', 'g')
vmax = max(sqrt(path_Uniform_BSpline1(:,1).*path_Uniform_BSpline1(:,1)+path_Uniform_BSpline1(:,2).*path_Uniform_BSpline1(:,2)))
vx = [];
vy = [];
for i = 2 : size(path_Uniform_BSpline(:,1),1)
    vx(i) = (path_Uniform_BSpline(i,1) - path_Uniform_BSpline(i-1,1))/(time(i) - time(i-1));
    vy(i) = (path_Uniform_BSpline(i,2) - path_Uniform_BSpline(i-1,2))/(time(i) - time(i-1));
end
[Vx, Time] = time_adjust(vx,time);
[Vy, Time] = time_adjust(vy,time);
plot(time,vx,'LineWidth', 3,  'Color', 'c')
% pos = cumtrapz(time,path_Uniform_BSpline1(:,1));
pos = cumtrapz(time,vx);
% cumtrapz(path_Uniform_BSpline(:,3),path_Uniform_BSpline1_old(:,1))
% plot(time,pos,'k--')
posi = pos(end)
plot(time,vy,'LineWidth', 2,  'Color', 'b')
% plot(time,cumtrapz(time,vy),'g--')
% figure(1)
% hold on
% grid on
[Px1, Time] = time_adjust(cumtrapz(time,path_Uniform_BSpline1(:,1))+3,time);
[Py1, Time] = time_adjust(cumtrapz(time,path_Uniform_BSpline1(:,2))+3,time);
figure
hold on
grid on
plot(Time,Px1,'LineWidth', 2,  'Color', 'k')
plot(Time,Py1,'LineWidth', 2,  'Color', 'k')
plot(Time,Px,'LineWidth', 3,  'Color', 'r')
plot(Time,Py,'LineWidth', 3,  'Color', 'g')
% for i = 2 : size(Time,2)
%     vxx(i) = (Px1(i) - Px1(i-1))/(Time(i) - Time(i-1));
%     vyy(i) = (Py1(i) - Py1(i-1))/(Time(i) - Time(i-1));
% end
% plot(Time,vxx,'k--');
% plot(Time,vyy,'m');

figure;
hold on
grid on
[Ax1, Time] = time_adjust(path_Uniform_BSpline2(:,1),time);
[Ay1, Time] = time_adjust(path_Uniform_BSpline2(:,2),time);
% plot(time,path_Uniform_BSpline2(:,1),'LineWidth', 3,  'Color', 'r')
% plot(time,path_Uniform_BSpline2(:,2),'LineWidth', 3,  'Color', 'g')
plot(Time,Ax1,'LineWidth', 3,  'Color', 'r')
plot(Time,Ay1,'LineWidth', 3,  'Color', 'g')
accmax = max(sqrt(path_Uniform_BSpline2(:,1).*path_Uniform_BSpline2(:,1)+...
path_Uniform_BSpline2(:,2).*path_Uniform_BSpline2(:,2)))
accx = [];
accy = [];
for i = 2 : size(vx,2)
    accx(i) = (vx(i) - vx(i-1))/(time(i) - time(i-1));
    accy(i) = (vy(i) - vy(i-1))/(time(i) - time(i-1));
end
% for i = 2 : size(vx,2)
%     accx(i) = (path_Uniform_BSpline1(i,1) - path_Uniform_BSpline1(i-1,1))/(time(i) - time(i-1));
%     accy(i) = (path_Uniform_BSpline1(i,2) - path_Uniform_BSpline1(i-1,2))/(time(i) - time(i-1));
% end
[Ax, Time] = time_adjust(accx,time);
[Ay, Time] = time_adjust(accy,time);
plot(time,accx,'LineWidth', 2,  'Color', 'c')
plot(time,accy,'LineWidth', 2,  'Color', 'b')
% plot(path_Uniform_BSpline2(:,1),path_Uniform_BSpline2(:,2));
% scatter(P2(1,:), P2(2,:), 40,'MarkerEdgeColor','k',...
%     'MarkerFaceColor','k');
% vel = cumtrapz(time(1:21),path_Uniform_BSpline2(1:21,1));
vel = cumtrapz(time,accx);
% plot(time,vel,'k--')
velo = vel(end)
% plot(time,cumtrapz(time,accy),'g--')

% figure
% hold on
% plot(Time,Px,Time,Px1,Time,Py,Time,Py1)
% plot(Time,Vx,Time,Vx1,Time,Vy,Time,Vy1)
% plot(Time,Ax,Time,Ax1,Time,Ay,Time,Ay1)
% save('CRRT_Bspline_Data.mat','Time','Px','Px1','Py','Py1','Vx','Vx1','Vy','Vy1','Ax','Ay','Ax1','Ay1');





% figure
% hold on
% grid on
% % plot(path_Quasi_Uniform_BSpline(:,1),path_Quasi_Uniform_BSpline(:,2),'LineWidth', 1,  'Color', 'b')
% plot(path_Quasi_Uniform_BSpline(:,3),path_Quasi_Uniform_BSpline(:,1),'LineWidth', 3,  'Color', 'r')
% plot(path_Quasi_Uniform_BSpline(:,3),path_Quasi_Uniform_BSpline(:,2),'LineWidth', 3,  'Color', 'g')
% 
% figure
% hold on
% grid on
% plot(path_Quasi_Uniform_BSpline1(:,3),path_Quasi_Uniform_BSpline1(:,1),'LineWidth', 3,  'Color', 'r')
% plot(path_Quasi_Uniform_BSpline1(:,3),path_Quasi_Uniform_BSpline1(:,2),'LineWidth', 3,  'Color', 'g')
% 
% vmax = max(sqrt(path_Quasi_Uniform_BSpline1(:,1).*path_Quasi_Uniform_BSpline1(:,1)+...
% path_Quasi_Uniform_BSpline1(:,2).*path_Quasi_Uniform_BSpline1(:,2)))
% delta_t = diff(path_Quasi_Uniform_BSpline(:,3));
% for i = 2 : size(path_Quasi_Uniform_BSpline(:,1),1)
%     vx(i) = (path_Quasi_Uniform_BSpline(i,1) - path_Quasi_Uniform_BSpline(i-1,1))...
%     /delta_t(1);
%     vy(i) = (path_Quasi_Uniform_BSpline(i,2) - path_Quasi_Uniform_BSpline(i-1,2))...
%     /delta_t(1);
% end
% plot(path_Quasi_Uniform_BSpline1(:,3),vx,'LineWidth', 2,  'Color', 'c')
% plot(path_Quasi_Uniform_BSpline1(:,3),vy,'LineWidth', 2,  'Color', 'b')
% 
% 
% figure
% hold on
% grid on
% plot(path_Quasi_Uniform_BSpline2(:,3),path_Quasi_Uniform_BSpline2(:,1),'LineWidth', 3,  'Color', 'r')
% plot(path_Quasi_Uniform_BSpline2(:,3),path_Quasi_Uniform_BSpline2(:,2),'LineWidth', 3,  'Color', 'g')
% accmax = max(sqrt(path_Quasi_Uniform_BSpline2(:,1).*path_Quasi_Uniform_BSpline2(:,1)+...
% path_Quasi_Uniform_BSpline2(:,2).*path_Quasi_Uniform_BSpline2(:,2)))
% 
% for i = 2 : size(path_Quasi_Uniform_BSpline(:,1),1)
%     accx(i) = (vx(i) - vx(i-1))...
%     /delta_t(1);
%     accy(i) = (vy(i) - vy(i))...
%     /delta_t(1);
% end
% plot(path_Quasi_Uniform_BSpline1(:,3),accx,'LineWidth', 2,  'Color', 'c')
% 
% function [idx] = getIdx(t,nodeVector)
%     for i = 1:length(nodeVector)-1
%         if nodeVector(i) <= t && t < nodeVector(i+1)
%             idx = i;
%             break;
%         end
%     end
% end

function [sequence_adjust,time_uniform] = time_adjust(sequence,time)
    % time_adjust = linspace(0, time(end), size(time,1));
    % time_adjust = [];
    step_size = 0.01;
    time_uniform = time(1):step_size:time(end);
    % 使用线性插值计算均匀时间序列对应的位置
    sequence_adjust = interp1(time, sequence, time_uniform, 'linear');
end

function [node_adjust] = node_limit(node,limit)
    node_adjust = node;
    for i = 1 : 1 : size(node,2)
        large = max(abs(node(:,i)));
        if large ~= 0
        if large > limit
            factor = large / limit;
            node_adjust(:,i) = node(:,i) / factor;
        % elseif large < limit
        % 
        end
        end
    end
end

function scaled_time = adjust_time(original_time, original_velocity, scaled_velocity)
    % 初始化缩放后的时间序列，先复制原始时间序列
    scaled_time = original_time;
    % 遍历每个时间点，根据缩放因子调整时间
    for i = 2:length(original_time)
        % 计算原始时间段的积分
        original_segment_integral = cumtrapz(original_time(i-1:i), original_velocity(i-1:i));
        original_segment_integral = original_segment_integral(end);
        % 计算缩放后时间段的积分
        scaled_segment_integral = cumtrapz(original_time(i-1:i), scaled_velocity(i-1:i));
        scaled_segment_integral = scaled_segment_integral(end);
        % 计算时间缩放因子
        time_scale_factor = original_segment_integral / scaled_segment_integral;
        % 调整时间
        scaled_time(i) = scaled_time(i-1) + (original_time(i) - original_time(i-1)) * time_scale_factor;
    end
end

function scaled_time = adjust_acc_time(original_time, original_velocity, scaled_velocity)
    % 初始化缩放后的时间序列，先复制原始时间序列
    scaled_time = original_time;
    % 遍历每个时间点，根据缩放因子调整时间
    for i = 2:length(original_time)
        % 计算原始时间段的积分
        original_segment_integral = cumtrapz(original_time(i-1:i), original_velocity(i-1:i));
        original_segment_integral = original_segment_integral(end);
        % 计算缩放后时间段的积分
        scaled_segment_integral = cumtrapz(original_time(i-1:i), scaled_velocity(i-1:i));
        scaled_segment_integral = scaled_segment_integral(end);
        % 计算时间缩放因子
        time_scale_factor = original_segment_integral / scaled_segment_integral;
        if scaled_segment_integral == 0
            time_scale_factor = 1;
        end
        % 调整时间
        scaled_time(i) = scaled_time(i-1) + (original_time(i) - original_time(i-1)) * sqrt(time_scale_factor);
    end
end
