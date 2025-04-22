% function plotRrtStarConnect(map_size,startPos, goalPos, map, treeNodes1,treeNodes2,path_opt)
close all
clear
% load('第五章实验\RRT_Star_Connect_V0.mat');
load('第五章实验\RRT_Star_Connect_V3.mat');
global path_opt path_Uniform_BSpline 
map = fun_defMap;
% 定义线段的起点和终点坐标
path_opt_old = path_opt;
control_point = [];
for idx = 1 : 1 : size(path_opt_old,1)-1
    start_point = path_opt_old(idx,:);
    end_point = path_opt_old(idx+1,:);
    % 计算线段的长度
    segment_length = norm(end_point - start_point);
    % 定义步长
    step = 5;
    if segment_length < step
        step_length = segment_length;
    else
        step_length = step;
    end
    % if segment_length < 5
    %     step_length = segment_length * 0.3;
    % else
    %     step_length = segment_length * 0.5;
    % end
    % 计算截取点的数量
    num_points = floor(segment_length / step_length) + 1;
    % 计算每个维度上的步长分量
    step_vector = (end_point - start_point) / (num_points - 1);
    % 初始化截取点矩阵
    points = zeros(num_points, 2);
    % 生成截取点
    for i = 1:num_points
        points(i, :) = start_point + (i - 1) * step_vector;
    end
    if idx ~= size(path_opt_old,1)-1
        points(end,:) = [];
    end
    control_point = [control_point; points];
end
path_opt = control_point;
% run('Bspline.m');
run('Bspline_and_deriv');
RRTC = figure;
%% 可视化地图
% 画起点和终点
scatter(startPos(1),startPos(2),80,'b','filled','pentagram')
hold on
scatter(goalPos(1),goalPos(2),80,'g','filled','pentagram')

% 画多边形障碍物
for i = 1:size(map.obs_polygon_boom,2)
    % per obs
    vertexs = map.obs_polygon_boom{i}{1};
    fill(vertexs(:,1),vertexs(:,2), [0.7 0.7 0.7],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
end

for i = 1:size(map.obs_polygon,2)-1
    % per obs
    vertexs = map.obs_polygon{i}{1};
    fill(vertexs(:,1),vertexs(:,2), [0.3 0.3 0.3],'EdgeColor', 'none', 'HandleVisibility', 'off'); hold on;
end
    vertexs = map.obs_polygon{size(map.obs_polygon,2)}{1};
    fill(vertexs(:,1),vertexs(:,2), [0.3 0.3 0.3],'EdgeColor', 'none'); hold on;

% 画圆形障碍物
t = linspace(0, 2*pi);
for i = 1:length(map.obs_circle_boom)-1
    x = cos(t)*map.obs_circle_boom(i,3)+map.obs_circle_boom(i,1);
    y = sin(t)*map.obs_circle_boom(i,3)+map.obs_circle_boom(i,2);
    fill(x,y,[0.7 0.7 0.7],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
end
    x = cos(t)*map.obs_circle_boom(length(map.obs_circle_boom),3)+map.obs_circle_boom(length(map.obs_circle_boom),1);
    y = sin(t)*map.obs_circle_boom(length(map.obs_circle_boom),3)+map.obs_circle_boom(length(map.obs_circle_boom),2);
    fill(x,y,[0.7 0.7 0.7],'EdgeColor', 'none'); hold on;

t = linspace(0, 2*pi);
for i = 1:length(map.obs_circle)-1
    x = cos(t)*map.obs_circle(i,3)+map.obs_circle(i,1);
    y = sin(t)*map.obs_circle(i,3)+map.obs_circle(i,2);
    fill(x,y,[0.3 0.3 0.3],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
end
    x = cos(t)*map.obs_circle(length(map.obs_circle),3)+map.obs_circle(length(map.obs_circle),1);
    y = sin(t)*map.obs_circle(length(map.obs_circle),3)+map.obs_circle(length(map.obs_circle),2);
    fill(x,y,[0.3 0.3 0.3],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;

axis equal
xlim([0 map_size(1)]);
ylim([0 map_size(2)]);

%% 画采样点与路径
for i = 2:size(treeNodes1,2)-1
    plot([treeNodes1(i).parentNode(1),treeNodes1(i).node(1)],...
        [treeNodes1(i).parentNode(2),treeNodes1(i).node(2)],'b','linewidth',0.5,'HandleVisibility', 'off');
end
    plot([treeNodes1(size(treeNodes1,2)).parentNode(1),treeNodes1(size(treeNodes1,2)).node(1)],...
        [treeNodes1(size(treeNodes1,2)).parentNode(2),treeNodes1(size(treeNodes1,2)).node(2)],'b','linewidth',0.5);
for i = 2:size(treeNodes2,2)-1
    plot([treeNodes2(i).parentNode(1),treeNodes2(i).node(1)],...
        [treeNodes2(i).parentNode(2),treeNodes2(i).node(2)],'g','linewidth',0.5,'HandleVisibility', 'off');
end
    plot([treeNodes2(size(treeNodes2,2)).parentNode(1),treeNodes2(size(treeNodes2,2)).node(1)],...
        [treeNodes2(size(treeNodes2,2)).parentNode(2),treeNodes2(size(treeNodes2,2)).node(2)],'g','linewidth',0.5);

plot(path_opt(:,1),path_opt(:,2),'r','linewidth',1.5)
plot(path_Uniform_BSpline(:,1),path_Uniform_BSpline(:,2),'c','linewidth',1)
ylabel('\fontname{宋体}距离\fontname{Times New Roman}(m)');
xlabel('\fontname{宋体}距离\fontname{Times New Roman}(m)');
% h = legend('\fontname{宋体}起点', '\fontname{宋体}目标点','\fontname{宋体}障碍物', ...
%     '\fontname{宋体}初始折线路径', ...
%     '\fontname{宋体}B样条平滑后的路径');
h = legend('\fontname{宋体}起点', '\fontname{宋体}目标点','\fontname{宋体}障碍物', ...
    '\fontname{宋体}障碍物膨胀边缘', '\fontname{Times New Roman}T_1\fontname{宋体}采样路径',...
    '\fontname{Times New Roman}T_2\fontname{宋体}采样路径','\fontname{宋体}初始折线路径', ...
    '\fontname{宋体}B样条平滑后的路径');
h.ItemTokenSize(1) = 15;
set(h,'NumColumns',1,'location','eastoutside','Box','off');


% sub_axes = axes('Position', [0.12, 0.54, 0.2, 0.2]); % [左, 下, 宽, 高]
% for i = 1:size(map.obs_polygon,2)
%     vertexs = map.obs_polygon{i}{1};
%     fill(vertexs(:,1),vertexs(:,2), [0.3 0.3 0.3],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
% end
% t = linspace(0, 2*pi);
% for i = 1:length(map.obs_circle)-1
%     x = cos(t)*map.obs_circle(i,3)+map.obs_circle(i,1);
%     y = sin(t)*map.obs_circle(i,3)+map.obs_circle(i,2);
%     fill(x,y,[0.3 0.3 0.3],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
% end
%     x = cos(t)*map.obs_circle(length(map.obs_circle),3)+map.obs_circle(length(map.obs_circle),1);
%     y = sin(t)*map.obs_circle(length(map.obs_circle),3)+map.obs_circle(length(map.obs_circle),2);
%     fill(x,y,[0.3 0.3 0.3],'EdgeColor', 'none'); hold on;
% 
% axis equal
% xlim([0 map_size(1)]);
% ylim([0 map_size(2)]);
% plot(path_opt(:,1),path_opt(:,2),'r','linewidth',1.5)
% plot(path_Uniform_BSpline(:,1),path_Uniform_BSpline(:,2),'c','linewidth',1)
% axis([18.7 23.3 13 16.3])   
% ax = gca;
% 
% % 设置坐标轴字号
% set(ax, 'FontSize', 7);
% % annotation('textarrow', [0.29, 0.23], [0.38, 0.50], 'FontSize', 10);
% annotation('textarrow', [0.27, 0.23], [0.37, 0.50], 'FontSize', 10);
% 
% 
% sub_axes = axes('Position', [0.4, 0.22, 0.2, 0.2]); % [左, 下, 宽, 高]
% for i = 1:size(map.obs_polygon,2)
%     vertexs = map.obs_polygon{i}{1};
%     fill(vertexs(:,1),vertexs(:,2), [0.3 0.3 0.3],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
% end
% t = linspace(0, 2*pi);
% for i = 1:length(map.obs_circle)-1
%     x = cos(t)*map.obs_circle(i,3)+map.obs_circle(i,1);
%     y = sin(t)*map.obs_circle(i,3)+map.obs_circle(i,2);
%     fill(x,y,[0.3 0.3 0.3],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
% end
%     x = cos(t)*map.obs_circle(length(map.obs_circle),3)+map.obs_circle(length(map.obs_circle),1);
%     y = sin(t)*map.obs_circle(length(map.obs_circle),3)+map.obs_circle(length(map.obs_circle),2);
%     fill(x,y,[0.3 0.3 0.3],'EdgeColor', 'none'); hold on;
% 
% axis equal
% xlim([0 map_size(1)]);
% ylim([0 map_size(2)]);
% plot(path_opt(:,1),path_opt(:,2),'r','linewidth',1.5)
% plot(path_Uniform_BSpline(:,1),path_Uniform_BSpline(:,2),'c','linewidth',1)
% axis([31 36 26.5 29.5])
% ax = gca;
% 
% % 设置坐标轴字号
% set(ax, 'FontSize', 7); 
% annotation('textarrow', [0.37, 0.45], [0.58, 0.44], 'FontSize', 10);

sub_axes = axes('Position', [0.43, 0.2, 0.2, 0.2]); % [左, 下, 宽, 高]
% 画多边形障碍物
for i = 1:size(map.obs_polygon_boom,2)
    % per obs
    vertexs = map.obs_polygon_boom{i}{1};
    fill(vertexs(:,1),vertexs(:,2), [0.7 0.7 0.7],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
end

for i = 1:size(map.obs_polygon,2)-1
    % per obs
    vertexs = map.obs_polygon{i}{1};
    fill(vertexs(:,1),vertexs(:,2), [0.3 0.3 0.3],'EdgeColor', 'none', 'HandleVisibility', 'off'); hold on;
end
    vertexs = map.obs_polygon{size(map.obs_polygon,2)}{1};
    fill(vertexs(:,1),vertexs(:,2), [0.3 0.3 0.3],'EdgeColor', 'none'); hold on;

% 画圆形障碍物
t = linspace(0, 2*pi);
for i = 1:length(map.obs_circle_boom)-1
    x = cos(t)*map.obs_circle_boom(i,3)+map.obs_circle_boom(i,1);
    y = sin(t)*map.obs_circle_boom(i,3)+map.obs_circle_boom(i,2);
    fill(x,y,[0.7 0.7 0.7],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
end
    x = cos(t)*map.obs_circle_boom(length(map.obs_circle_boom),3)+map.obs_circle_boom(length(map.obs_circle_boom),1);
    y = sin(t)*map.obs_circle_boom(length(map.obs_circle_boom),3)+map.obs_circle_boom(length(map.obs_circle_boom),2);
    fill(x,y,[0.7 0.7 0.7],'EdgeColor', 'none'); hold on;

t = linspace(0, 2*pi);
for i = 1:length(map.obs_circle)-1
    x = cos(t)*map.obs_circle(i,3)+map.obs_circle(i,1);
    y = sin(t)*map.obs_circle(i,3)+map.obs_circle(i,2);
    fill(x,y,[0.3 0.3 0.3],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
end
    x = cos(t)*map.obs_circle(length(map.obs_circle),3)+map.obs_circle(length(map.obs_circle),1);
    y = sin(t)*map.obs_circle(length(map.obs_circle),3)+map.obs_circle(length(map.obs_circle),2);
    fill(x,y,[0.3 0.3 0.3],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;

axis equal
xlim([0 map_size(1)]);
ylim([0 map_size(2)]);

%% 画采样点与路径
for i = 2:size(treeNodes1,2)-1
    plot([treeNodes1(i).parentNode(1),treeNodes1(i).node(1)],...
        [treeNodes1(i).parentNode(2),treeNodes1(i).node(2)],'b','linewidth',0.5,'HandleVisibility', 'off');
end
    plot([treeNodes1(size(treeNodes1,2)).parentNode(1),treeNodes1(size(treeNodes1,2)).node(1)],...
        [treeNodes1(size(treeNodes1,2)).parentNode(2),treeNodes1(size(treeNodes1,2)).node(2)],'b','linewidth',0.5);
for i = 2:size(treeNodes2,2)-1
    plot([treeNodes2(i).parentNode(1),treeNodes2(i).node(1)],...
        [treeNodes2(i).parentNode(2),treeNodes2(i).node(2)],'g','linewidth',0.5,'HandleVisibility', 'off');
end
    plot([treeNodes2(size(treeNodes2,2)).parentNode(1),treeNodes2(size(treeNodes2,2)).node(1)],...
        [treeNodes2(size(treeNodes2,2)).parentNode(2),treeNodes2(size(treeNodes2,2)).node(2)],'g','linewidth',0.5);

plot(path_opt(:,1),path_opt(:,2),'r','linewidth',1.5)
plot(path_Uniform_BSpline(:,1),path_Uniform_BSpline(:,2),'c','linewidth',1)
axis([33 40.5 14.5 19])
annotation('textarrow', [0.405, 0.445], [0.40, 0.38], 'FontSize', 10);

set(RRTC.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', ...
    1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
ax = gca;
set(ax, 'FontSize', 7); 


fig = gcf;
fig.Units = 'centimeters';
fig.Position = [5 10 15 7.45]; 
% fimename = 'D:\Flycontrol\SCUT_thesis\Fig\B样条曲线路径平滑实验结果.pdf';
% PlotToFileColorPDF(RRTC,fimename,15,7.45);
% end