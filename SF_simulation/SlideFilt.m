function [Dat, Filt] = SlideFilt(Dat, DatRel, num, Filt, Cmd)
    % 滑窗均值滤波器
    % Dat: 当前数据数组
    % DatRel: 新数据数组
    % num: 数据的数量
    % Filt: 滤波器结构体，包含CNT和CCR
    % Cmd: 控制命令 (0-重新初始化，1-正常滤波，2-使用同个FIL滤波)

    switch Cmd
        case 0
            Filt.CNT = 1;
        case 1
            if Filt.CNT < Filt.CCR
                Filt.CNT = Filt.CNT + 1;
            end
        case 2
            % 什么也不做
    end

    if Filt.CNT == 1
        Dat(1:num) = DatRel(1:num);
    end

    for i = 1:num
        Dat(i) = (Dat(i) * (Filt.CNT - 1) + DatRel(i)) / Filt.CNT;
    end

    % 返回修改后的Dat和Filt
    return
end
