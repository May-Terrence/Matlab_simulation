classdef PID < handle
    properties
        lastError = 0;
        error = 0;
        integral = 0;
        pout;
        iout;
        dout = 0;
        dout2 = 0;
        dout3 = 0;
        output = 0;
        Kp = 0; 
        Ki = 0; 
        Kd = 0; 
        iLimit = 0; 
        ERRDEF_SLOPE_NUM = 20; 
        ErrDefTmp = zeros(1,20);
    end
    
    methods
                % 构造函数
        function obj = PID(Kp,Ki,Kd,iLimit)
            % 对象属性初始化
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.iLimit = iLimit;
        end
        function result = PID_Controller(obj, err, Dt)
            obj.lastError = obj.error;
            obj.error = err;
            if((obj.lastError>0 && obj.error<0) || (obj.lastError<0 && obj.error>0))
                obj.ErrDefTmp = zeros(1, obj.ERRDEF_SLOPE_NUM);
            end
            obj.pout = obj.error * obj.Kp;
            obj.integral = obj.integral + obj.error * Dt;
            obj.iout = fConstrain(obj.integral * obj.Ki, -obj.iLimit, obj.iLimit);
            obj.dout = (obj.error - obj.lastError) / Dt;
            for i = 2:obj.ERRDEF_SLOPE_NUM
                obj.ErrDefTmp(i-1) = obj.ErrDefTmp(i);
            end
            obj.ErrDefTmp(obj.ERRDEF_SLOPE_NUM) = obj.error;
            ab = polyfit(1:obj.ERRDEF_SLOPE_NUM, obj.ErrDefTmp, 1);
            obj.dout2 = ab(1) / Dt;
            obj.dout3 = obj.dout2 * obj.Kd;
            if (obj.pout > 0 && obj.dout3 < 0) || (obj.pout < 0 && obj.dout3 > 0)
                obj.dout3 = 0;
            end
%             obj.dout3 = obj.dout * obj.Kd;
%             if (obj.pout > 0 && obj.dout < 0) || (obj.pout < 0 && obj.dout > 0)
%                 obj.dout3 = 0;
%             end
            obj.output = obj.pout + obj.iout + obj.dout3;
            result.output = obj.output;
            result.pout = obj.pout;
            result.iout = obj.iout;
            result.dout = obj.dout3;
            result.lastError = obj.lastError;
            result.ErrDefTmp = obj.ErrDefTmp;
        end
        function show(obj)
             disp(['obj.pout = ', num2str(obj.pout)])
             disp(['obj.iout = ', num2str(obj.iout)])
             disp(['obj.dout3 = ', num2str(obj.dout3)])
             disp(['obj.out = ', num2str(obj.output)])
        end
    end
end

function value = fConstrain(x, min_val, max_val)
    value = max(min(x, max_val), min_val);
end
