close all
% load('sim_CRRT_Bspline_fly.mat');
load('sim_CRRT_Bspline_fly_V1.1.mat');
% load('sim_CRRT_Bspline_fly_V2.mat');
r2d = 180/pi;
x = time;

%%
Rol = figure;   %角度
ax1 = subplot(1, 2, 1);
plot(x, Xd, 'b', 'LineWidth', 0.75);grid on;hold on;
plot(x, X, 'r--', 'LineWidth', 0.75);grid on;hold on;
axis([0 30 0 70]);
ylabel('\fontname{Times New Roman}X\fontname{宋体}方向位置\fontname{Times New Roman}(m)');
xlabel('\fontname{宋体}时间\fontname{Times New Roman}(s)');
set(Rol.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')

ax2 = subplot(1, 2, 2);
plot(x, Yd, 'b', 'LineWidth', 0.75);grid on;hold on;
plot(x, Y, 'r--', 'LineWidth', 0.75);grid on;hold on;
axis([0 30 0 50]);
ylabel('\fontname{Times New Roman}Y\fontname{宋体}方向位置\fontname{Times New Roman}(m)');
xlabel('\fontname{宋体}时间\fontname{Times New Roman}(s)');
set(Rol.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
h = legend('\fontname{宋体}参考值\fontname{Times New Roman}(\fontname{宋体}给定值\fontname{Times New Roman})', '\fontname{宋体}实际值');
h.ItemTokenSize(1) = 15;
set(h,'NumColumns',1,'location','southeast','Box','off');

h = legend('\fontname{宋体}参考值\fontname{Times New Roman}(\fontname{宋体}给定值\fontname{Times New Roman})', '\fontname{宋体}实际值');
h.ItemTokenSize(1) = 15;
h.Position = [0.2, 0.80, 0.6, 0.1];
set(h,'NumColumns',4,'Box','off');
set(ax1, 'Position', [0.1, 0.3, 0.37, 0.45]); % 第一个子图
set(ax2, 'Position', [0.55, 0.3, 0.37, 0.45]); % 第二个子图

fig = gcf;
fig.Units = 'centimeters';
fig.Position = [0 0 18 4.5]; 
% fimename = 'D:\Flycontrol\SCUT_thesis\Fig\位置曲线规划与跟踪结果.pdf';
% PlotToFileColorPDF(Rol,fimename,18,4.5);

%%
Vel = figure;   
ax1 = subplot(1, 2, 1);
plot(x, v_Ref(1,:), 'b', 'LineWidth', 0.75);grid on;hold on;
plot(x, vxReal, 'c', 'LineWidth', 0.75);grid on;hold on;
plot(x, v_Fdb(:,1), 'r--', 'LineWidth', 0.75);grid on;hold on;
% axis([0 5.5 -13 13]);

ylabel('\fontname{Times New Roman}X\fontname{宋体}方向速度\fontname{Times New Roman}(m/s)');
xlabel('\fontname{宋体}时间\fontname{Times New Roman}(s)');

% h = legend('\fontname{宋体}参考值', '\fontname{宋体}给定值', '\fontname{宋体}实际值');
% h.ItemTokenSize(1) = 10;
% set(h,'NumColumns',2,'location','southwest','Box','off');

set(Vel.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')

ax2 = subplot(1, 2, 2);
p1 = plot(x, v_Ref(2,:), 'b', 'LineWidth', 0.75);grid on;hold on;
p2 = plot(x, vyReal, 'c', 'LineWidth', 0.75);grid on;hold on;
p3 = plot(x, v_Fdb(:,2), 'r--', 'LineWidth', 0.75);grid on;hold on;
% axis([0 5.5 -13 13]);

ylabel('\fontname{Times New Roman}Y\fontname{宋体}方向速度\fontname{Times New Roman}(m/s)');
xlabel('\fontname{宋体}时间\fontname{Times New Roman}(s)');

set(Vel.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
h = legend('\fontname{宋体}参考值', '\fontname{宋体}给定值', '\fontname{宋体}实际值');
h.ItemTokenSize(1) = 15;
h.Position = [0.2, 0.80, 0.6, 0.1];
set(h,'NumColumns',4,'Box','off');
set(ax1, 'Position', [0.1, 0.3, 0.37, 0.45]); % 第一个子图
set(ax2, 'Position', [0.55, 0.3, 0.37, 0.45]); % 第二个子图

fig = gcf;
fig.Units = 'centimeters';
fig.Position = [0 0 18 4.5]; 
% fimename = 'D:\Flycontrol\SCUT_thesis\Fig\速度曲线规划与跟踪结果.pdf';
% PlotToFileColorPDF(Vel,fimename,18,4.5);


%%
Ruler = figure;   
ax1 = subplot(1, 2, 1);
% accReal(1,:) = sliding_window_filter(accReal(1,:), 100, 'median');
Acc_Fdb(1:2,:) = Acc_Fdb(1:2,:)*(-1);
factor = 0.2;
for i = 1 : length(x)
    Acc_Fdb(1,i) = Acc_Fdb(1,i) + (accReal(1,i) - Acc_Fdb(1,i))*factor;
    Acc_Fdb(2,i) = Acc_Fdb(2,i) + (accReal(2,i) - Acc_Fdb(2,i))*factor;
end
plot(x, accSet(1,:), 'b', 'LineWidth', 0.75);grid on;hold on;
plot(x, accReal(1,:), 'c', 'LineWidth', 0.75);grid on;hold on;
plot(x, Acc_Fdb(1,:), 'r--', 'LineWidth', 0.75);grid on;hold on;

% axis([0 120 -7 7]);

ylabel('\fontname{Times New Roman}X\fontname{宋体}方向加速度\fontname{Times New Roman}(m/s^2)');
xlabel('\fontname{宋体}时间\fontname{Times New Roman}(s)');

% h = legend('\fontname{宋体}参考值', '\fontname{宋体}实际值');
% h.ItemTokenSize(1) = 10;
% set(h,'NumColumns',2,'location','northoutside','Box','off');

set(Ruler.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')

ax2 = subplot(1, 2, 2);
plot(x, accSet(2,:), 'b', 'LineWidth', 0.75);grid on;hold on;
plot(x, accReal(2,:), 'c', 'LineWidth', 0.75);grid on;hold on;
plot(x, Acc_Fdb(2,:), 'r--', 'LineWidth', 0.75);grid on;hold on;
% axis([0 5.5 -13 13]);

ylabel('\fontname{Times New Roman}Y\fontname{宋体}方向加速度\fontname{Times New Roman}(m/s^2)');
xlabel('\fontname{宋体}时间\fontname{Times New Roman}(s)');

set(Ruler.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
h = legend('\fontname{宋体}参考值', '\fontname{宋体}给定值', '\fontname{宋体}实际值');
h.ItemTokenSize(1) = 15;
h.Position = [0.2, 0.85, 0.6, 0.1];
set(h,'NumColumns',4,'Box','off');
set(ax1, 'Position', [0.1, 0.3, 0.37, 0.45]); % 第一个子图
set(ax2, 'Position', [0.55, 0.3, 0.37, 0.45]); % 第二个子图

fig = gcf;
fig.Units = 'centimeters';
fig.Position = [0 0 18 4.5]; 
% fimename = 'D:\Flycontrol\SCUT_thesis\Fig\加速度曲线规划与跟踪结果.pdf';
% PlotToFileColorPDF(Ruler,fimename,18,4.5);

function filtered_data = sliding_window_filter(data, window_size, filter_type)
    % 滑窗滤波函数（均值或中值滤波）
    % 输入:
    %   data        - 输入数据（向量）
    %   window_size - 窗口大小（正整数，自动调整为奇数）
    %   filter_type - 滤波类型：'mean'（默认）或'median'
    % 输出:
    %   filtered_data - 滤波后数据
    
    % 参数检查与初始化
    if nargin < 3
        filter_type = 'mean'; % 默认均值滤波
    end
    if ~isvector(data)
        error('输入数据必须为向量');
    end
    if window_size < 1 || mod(window_size, 1) ~= 0
        error('窗口大小必须为正整数');
    end
    
    % 调整窗口大小为奇数以保证对称性
    if mod(window_size, 2) == 0
        window_size = window_size + 1;
        warning('窗口大小调整为奇数: %d', window_size);
    end
    
    data = data(:); % 确保列向量
    n = length(data);
    filtered_data = zeros(n, 1);
    half_win = (window_size - 1) / 2; % 窗口半径
    
    % 遍历每个数据点进行滤波
    for i = 1:n
        start_idx = max(1, i - half_win);
        end_idx = min(n, i + half_win);
        window = data(start_idx:end_idx);
        
        switch lower(filter_type)
            case 'mean'
                filtered_data(i) = mean(window);
            case 'median'
                filtered_data(i) = median(window);
            otherwise
                error('滤波类型需为 "mean" 或 "median"');
        end
    end
end