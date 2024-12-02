% 根据型值点反求控制点
function P = dataPoint2ControlPoint(Q)
Q = Q';

% 构造n×6矩阵
n = size(Q,1);
A = zeros(n,n);
for i = 1:n
    if i == 1
        A(i,1) = 6;
    elseif i == n
        A(i,n) = 6;
    else
        A(i,i-1) = 1;
        A(i,i) = 4;
        A(i,i+1) = 1;
    end
end

% 矩阵运算
P_x = A\Q(:,1)*6;
P_y = A\Q(:,2)*6;
P = [P_x,P_y];
P0 = 2*P(1,:) - P(2,:);
P_n_plus_1 = 2*P(n,:) - P(n-1,:);
P = [P0; P; P_n_plus_1];
P = P';
end
