function [Dat, Filt] = SlideFilt(Dat, DatRel, num, Filt, Cmd)
    % ������ֵ�˲���
    % Dat: ��ǰ��������
    % DatRel: ����������
    % num: ���ݵ�����
    % Filt: �˲����ṹ�壬����CNT��CCR
    % Cmd: �������� (0-���³�ʼ����1-�����˲���2-ʹ��ͬ��FIL�˲�)

    switch Cmd
        case 0
            Filt.CNT = 1;
        case 1
            if Filt.CNT < Filt.CCR
                Filt.CNT = Filt.CNT + 1;
            end
        case 2
            % ʲôҲ����
    end

    if Filt.CNT == 1
        Dat(1:num) = DatRel(1:num);
    end

    for i = 1:num
        Dat(i) = (Dat(i) * (Filt.CNT - 1) + DatRel(i)) / Filt.CNT;
    end

    % �����޸ĺ��Dat��Filt
    return
end
