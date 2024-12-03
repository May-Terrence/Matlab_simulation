% ����B�������ߵĻ�ͼ
% clc
% clear
% close all
global path_opt path_Quasi_Uniform_BSpline path_len
%% ���ݶ���
k = 4;                                    % k�ס�k-1��B����
P = [0, 1, 2, 3, 4;
    0, 1, 0, 1, 0];                       % 5�����Ƶ㣬����������������
% P = [P(:,1), P(:,1), P(:,:), P(:,end), P(:,end)];
P = path_opt';
P = [P(:,1), P(:,:), P(:,end)];
n = size(P,2) - 1;                          % n�ǿ��Ƶ��������0��ʼ����

%% ����B��������
path = [];
Bik = zeros(n+1, 1);
for i = 2
    nodeVector = getNodeVector(n, k, i);
    if i == 1   % ����B����
        path_Uniform_BSpline = [];
        for u = (k-1)/(n+k) : 0.001 : 1-(k-1)/(n+k)  %(k-1)/(n+k+1) : 0.001 : (n+2)/(n+k+1)
            for j = 0 : 1 : n
                Bik(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector);
            end
            p_u = P * Bik;
            path_Uniform_BSpline(end+1,:) = p_u;
        end
    elseif i == 2 % ׼����B����
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
        % ���ж��Ƿ�����ֶα��������ߵĻ���Ҫ��
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
            fprintf('������ֶα���������Ҫ��!\n');
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
%% ����
% save path_BSpline.mat  P ...
%     path_Uniform_BSpline...
%     path_Quasi_Uniform_BSpline... 
%     path_Piecewise_Bezier...
%     path_Nonuniform_BSpline

%% ��ͼ

% ������B��������
% figure
% hold on
% grid on
% % ����ͼ�λ���
% plot(P(1,:), P(2,:),'LineWidth', 3,  'Color', 'b');
% plot(path_Uniform_BSpline(:,1),path_Uniform_BSpline(:,2),'LineWidth', 3,  'Color', 'r')
% scatter(P(1,:), P(2,:), 40,'MarkerEdgeColor','g',...
%     'MarkerFaceColor','g');
% % ������
% set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
% hXLabel = xlabel('x');
% hYLabel = ylabel('y');
% % �޸Ŀ̶ȱ�ǩ������ֺ�
% set(gca, 'FontSize', 16),...
% set([hXLabel, hYLabel], 'FontName',  'simsun')
% set([hXLabel, hYLabel], 'FontSize', 16)

% ��׼����B��������
% figure
% hold on
% grid on
% % ����ͼ�λ���
% plot(P(1,:), P(2,:),'-','LineWidth', 3,  'Color', [.3, .6, .9]);
% plot(path_Quasi_Uniform_BSpline(:,1),path_Quasi_Uniform_BSpline(:,2),'LineWidth', 1,  'Color', 'r')
% scatter(P(1,:), P(2,:), 40,'MarkerEdgeColor','g','MarkerFaceColor','g');
% % ������
% set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
% hXLabel = xlabel('x');
% hYLabel = ylabel('y');
% % �޸Ŀ̶ȱ�ǩ������ֺ�
% set(gca, 'FontSize', 16),...
% set([hXLabel, hYLabel], 'FontName',  'simsun')
% set([hXLabel, hYLabel], 'FontSize', 16)

% % ����α���������
% figure
% hold on
% grid on
% % ����ͼ�λ���
% plot(P(1,:), P(2,:),'LineWidth', 3,  'Color', 'b');
% plot(path_Piecewise_Bezier(:,1),path_Piecewise_Bezier(:,2),'LineWidth', 3,  'Color', 'r')
% scatter(P(1,:), P(2,:),  40,'MarkerEdgeColor','g',...
%     'MarkerFaceColor','g');
% 
% % ������
% set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
% hXLabel = xlabel('x');
% hYLabel = ylabel('y');
% % �޸Ŀ̶ȱ�ǩ������ֺ�
% set(gca, 'FontSize', 16),...
% set([hXLabel, hYLabel], 'FontName',  'simsun')
% set([hXLabel, hYLabel], 'FontSize', 16)
% 
% % ���Ǿ���B��������
% figure
% hold on
% grid on
% % ����ͼ�λ���
% plot(P(1,:), P(2,:),'LineWidth', 3,  'Color', 'b');
% plot(path_Nonuniform_BSpline(:,1),path_Nonuniform_BSpline(:,2),'LineWidth', 3,  'Color', 'r')
% scatter(P(1,:), P(2,:),  40,'MarkerEdgeColor','g',...
%     'MarkerFaceColor','g');
% 
% % ������
% set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
% hXLabel = xlabel('x');
% hYLabel = ylabel('y');
% % �޸Ŀ̶ȱ�ǩ������ֺ�
% set(gca, 'FontSize', 16),...
% set([hXLabel, hYLabel], 'FontName',  'simsun')
% set([hXLabel, hYLabel], 'FontSize', 16)