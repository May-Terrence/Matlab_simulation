% 四类B样条曲线的画图
% clc
% clear
% close all
global path_opt path_Quasi_Uniform_BSpline path_len
%% 数据定义
k = 4;                                    % k阶、k-1次B样条
P = [0, 1, 2, 3, 4;
    0, 1, 0, 1, 0];                       % 5个控制点，可以满足曲率连续
% P = [P(:,1), P(:,1), P(:,:), P(:,end), P(:,end)];
P = path_opt';
P = [P(:,1), P(:,:), P(:,end)];
n = size(P,2) - 1;                          % n是控制点个数，从0开始计数

%% 生成B样条曲线
path = [];
Bik = zeros(n+1, 1);
for i = 2
    nodeVector = getNodeVector(n, k, i);
    if i == 1   % 均匀B样条
        path_Uniform_BSpline = [];
        for u = (k-1)/(n+k) : 0.001 : 1-(k-1)/(n+k)  %(k-1)/(n+k+1) : 0.001 : (n+2)/(n+k+1)
            for j = 0 : 1 : n
                Bik(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector);
            end
            p_u = P * Bik;
            path_Uniform_BSpline(end+1,:) = p_u;
        end
    elseif i == 2 % 准均匀B样条
        path_Quasi_Uniform_BSpline = [];
        delta = 0.02 / path_len;
%         for u = 0 : 0.005 : 1-0.005
        for u = 0 : delta : 1 - delta
            for j = 0 : 1 : n
                Bik(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector);
            end
            p_u = P * Bik;
            path_Quasi_Uniform_BSpline(end+1,:) = p_u;
        end
    elseif i == 3
        % 先判断是否满足分段贝塞尔曲线的基本要求
        path_Piecewise_Bezier = [];
        if mod(n,k-1) == 0
            for u = 0 : 0.005 : 1-0.005
                for j = 0 : 1 : n
                    Bik(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector);
                end
                p_u = P * Bik;
                path_Piecewise_Bezier(end+1,:) = p_u;
            end
        else
            fprintf('不满足分段贝塞尔曲线要求!\n');
        end
    else
        path_Nonuniform_BSpline = [];
        for u = 0 : 0.005 : 1-0.005
            for j = 0 : 1 : n
                Bik(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector);
            end
            p_u = P * Bik;
            path_Nonuniform_BSpline(end+1,:) = p_u;
        end
    end
end
%% 保存
% save path_BSpline.mat  P ...
%     path_Uniform_BSpline...
%     path_Quasi_Uniform_BSpline... 
%     path_Piecewise_Bezier...
%     path_Nonuniform_BSpline

%% 画图

% 画均匀B样条曲线
% figure
% hold on
% grid on
% % 主体图形绘制
% plot(P(1,:), P(2,:),'LineWidth', 3,  'Color', 'b');
% plot(path_Uniform_BSpline(:,1),path_Uniform_BSpline(:,2),'LineWidth', 3,  'Color', 'r')
% scatter(P(1,:), P(2,:), 40,'MarkerEdgeColor','g',...
%     'MarkerFaceColor','g');
% % 坐标轴
% set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
% hXLabel = xlabel('x');
% hYLabel = ylabel('y');
% % 修改刻度标签字体和字号
% set(gca, 'FontSize', 16),...
% set([hXLabel, hYLabel], 'FontName',  'simsun')
% set([hXLabel, hYLabel], 'FontSize', 16)

% 画准均匀B样条曲线
% figure
% hold on
% grid on
% % 主体图形绘制
% plot(P(1,:), P(2,:),'-','LineWidth', 3,  'Color', [.3, .6, .9]);
% plot(path_Quasi_Uniform_BSpline(:,1),path_Quasi_Uniform_BSpline(:,2),'LineWidth', 1,  'Color', 'r')
% scatter(P(1,:), P(2,:), 40,'MarkerEdgeColor','g','MarkerFaceColor','g');
% % 坐标轴
% set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
% hXLabel = xlabel('x');
% hYLabel = ylabel('y');
% % 修改刻度标签字体和字号
% set(gca, 'FontSize', 16),...
% set([hXLabel, hYLabel], 'FontName',  'simsun')
% set([hXLabel, hYLabel], 'FontSize', 16)

% % 画多段贝塞尔曲线
% figure
% hold on
% grid on
% % 主体图形绘制
% plot(P(1,:), P(2,:),'LineWidth', 3,  'Color', 'b');
% plot(path_Piecewise_Bezier(:,1),path_Piecewise_Bezier(:,2),'LineWidth', 3,  'Color', 'r')
% scatter(P(1,:), P(2,:),  40,'MarkerEdgeColor','g',...
%     'MarkerFaceColor','g');
% 
% % 坐标轴
% set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
% hXLabel = xlabel('x');
% hYLabel = ylabel('y');
% % 修改刻度标签字体和字号
% set(gca, 'FontSize', 16),...
% set([hXLabel, hYLabel], 'FontName',  'simsun')
% set([hXLabel, hYLabel], 'FontSize', 16)
% 
% % 画非均匀B样条曲线
% figure
% hold on
% grid on
% % 主体图形绘制
% plot(P(1,:), P(2,:),'LineWidth', 3,  'Color', 'b');
% plot(path_Nonuniform_BSpline(:,1),path_Nonuniform_BSpline(:,2),'LineWidth', 3,  'Color', 'r')
% scatter(P(1,:), P(2,:),  40,'MarkerEdgeColor','g',...
%     'MarkerFaceColor','g');
% 
% % 坐标轴
% set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
% hXLabel = xlabel('x');
% hYLabel = ylabel('y');
% % 修改刻度标签字体和字号
% set(gca, 'FontSize', 16),...
% set([hXLabel, hYLabel], 'FontName',  'simsun')
% set([hXLabel, hYLabel], 'FontSize', 16)