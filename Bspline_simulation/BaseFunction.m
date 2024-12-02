% ����B����������
function Bik_u = BaseFunction(i, k , u, NodeVector)
if k == 0       % 0��B����
    if u >= NodeVector(i+1) && u < NodeVector(i+2)
        Bik_u = 1;
    else
        Bik_u = 0;
    end
else
    % ֧������ĳ���
    length1 = NodeVector(i+k+1) - NodeVector(i+1);
    length2 = NodeVector(i+k+2) - NodeVector(i+2);     
    
     % �涨0/0 = 0
    if length1 == 0      
        length1 = 1;
    end
    if length2 == 0
        length2 = 1;
    end
    
    % ����������
    Bik_u = (u - NodeVector(i+1)) / length1 * BaseFunction(i, k-1, u, NodeVector) ...
        + (NodeVector(i+k+2) - u) / length2 * BaseFunction(i+1, k-1, u, NodeVector);
end
