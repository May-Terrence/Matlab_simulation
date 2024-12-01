function is_intersect = intersect_check(A,B,V1,V2)
% 检测线段AB与线段V12是否相交
% 判断V1,V2与线段AB的位置
AB = B - A;
AV1 = V1 - A;
AV2 = V2 - A;
result1 = AB(1) * AV1(2) - AB(2) * AV1(1);
result2 = AB(1) * AV2(2) - AB(2) * AV2(1);
if result1~=0 && result2~=0
    % 同号
    if result1*result2 > 0
        is_intersect = false;
        return;
    end
    % 两线段共线(result1=result2=0)视为不相交
    % 某点在线段上(result1=0 || result2=0)视为不相交
else
    is_intersect = false;
    return;
end
% 判断A,B与线段V12的位置
V12 = V2 - V1;
V1A = A - V1;
V1B = B - V1;
result1 = V12(1) * V1A(2) - V12(2) * V1A(1);
result2 = V12(1) * V1B(2) - V12(2) * V1B(1);
if result1~=0 && result2~=0
    % 同号
    if result1*result2 > 0
        is_intersect = false;
        return;
    end
    % 两线段共线(result1=result2=0)视为不相交
    % 某点在线段上(result1=0 || result2=0)视为不相交
else
    is_intersect = false;
    return;
end
is_intersect = true;
end