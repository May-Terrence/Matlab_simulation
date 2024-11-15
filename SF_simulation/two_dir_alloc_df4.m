function [u]=two_dir_alloc_df4(B,v_T, v_D,umin,umax)
% (c) mengchaoheng
% Last edited 2019-11
v=v_T+v_D;
v = [0;0;1];
[uv,z_uv,~]  = allocator_dir_LPwrap_4(B,v,umin,umax); % wls_alloc_mch(v,u);% 先计算合力矩所需舵量
if(z_uv>=1) % 若舵量可以满足则直接满足
    u=uv;
else  % 否则再计算扰动所需
     [uv1,z_uv1,~]= allocator_dir_LPwrap_4(B,v_T,umin,umax); % wls_alloc_mch(v1,u);
    if(z_uv1>=1)  % 若扰动可满足，合力矩不能满足，则进行两次分配
        umin1=umin-uv1;
        umax1=umax-uv1;
        uv2 = allocator_dir_LPwrap_4(B,v_D,umin1,umax1);
        u=uv1+uv2;
    else  % 扰动也不能满足，则直接按照合力矩进行分配
        u=uv;
    end
end
end

function [u,z,iters] = allocator_dir_LPwrap_4(B, v, umin,umax)
%%  come from [u] = LPwrap(IN_MAT) and using sigle data, define k, m for df4
% [k,m]=size(B);

itlim=uint16(5e2);%迭代最大次数
output_size = uint8(4);
[u,itlim,~,z] = DPscaled_LPCA_C(v,B,umin,umax,itlim,output_size);
iters=uint16(5e2)-itlim;
end


function [u,itlim,errout,rho] = DPscaled_LPCA_C(yd,B,uMin,uMax,itlim,m)
% Direction Preserving Control Allocation Linear Program
%     Reduced formulation (Solution Scaled from Boundary)
%
% function [u,itlim,errout] = DPscaled_LPCA(yd,B,uMin,uMax,itlim);
%
%    Solves the control allocation problem while preserving the
%  objective direction for unattainable commands. The reduced
%  dimension of the linear program passed to the Bounded Revised
%  Simplex solver is formed by forcing the solution to be on the
%  boundary of the AMS and eliminating the highest magnitude
%  objective by solving the other constraints in terms of it.
%
%  For yd outside the AMS, the solution returned is that the
%  maximum in the direction of yd
%    B*u= lamda*yd
%    max lamda s.t. uMin <= u <= uMax
%
%  Reducing the degrees of freedom elminates the problems of redundant
%  solutions for attainable objectives. If the desired objective is on the
%  interior of the AMS the solution is scaled from the solution on the
%  boundary, yielding the same controls as the Direct Allocation solution.
%  
%  (In the text this solution is discussed in section A.5.3)
%
%   (See Bodson, M., "Evaluation of Optimization Methods for
%          Control Allocation",  AIAA 2001-4223).
%
%  Inputs:
%          yd [n]    = Desired objective
%          B [n,m]   = Control Effectiveness matrix
%          uMin[m,1] = Lower bound for controls
%          uMax[m,1] = Upper bound for controls
%          itlim     = Number of allowed iterations limit
%                         (Sum of iterations in both branches)
%
% Outputs:
%         u[m,1]     = Control Solution
%         errout     = Error Status code
%                         0 = found solution
%                         <0 = Error in finding initial basic feasible solution
%                         >0 = Error in finding final solution
%                         -1,1 = Solver error (unbounded solution)
%                         -2   = Initial feasible solution not found
%                         -3,3 = Iteration limit exceeded
%         itlim      = Number of iterations remaining after solution found
%
% Calls:
%         simplxuprevsol_C = Bounded Revised Simplex solver (simplxuprevsol_C.m)
%
% Notes:
%    If yd is close to zero, u = 0;
%
%    Error code < 0 implies an error in the initialization and there is no guarantee on
%  the quality of the output solution other than the control limits.
%    Error code > 0 for errors in final solution.
%
% Modification History
%   2002      Roger Beck  Original ( DPcaLP2.m)
%   8/2014    Roger Beck  Update


%Initialize error code to zero
errout = int8(0);
rho=single(0);
%Figure out how big the problem is (use standard CA definitions for m & n)
% [n,m] = size(B);
n=uint8(3);%3维输入
tol=single(1e-8);

% Locate the maximum magnitude element in the desired objective
[my,iy]=max(abs(yd));

%Trivial solution, if desired moment is close to zero
%  May want to adjust the tolerance to improve numerics of later steps
if (my < tol)    %yd = 0 ==> u=0
    errout = int8(-1);  %Set flag to let caller know that it wasn't solved
    u = single(zeros(m,1));
    return;
end

%Transform Problem by Reordering Objectives with maximum first
Bt = B([iy setdiff(1:n,iy)],:);
ydt = yd([iy setdiff(1:n,iy)]);
ydt(2:3) = ydt([3 2]);
Bt([2 3],:) = Bt([3 2],:);
%%Convert into a LP problem
M = [ydt(2:n) -ydt(1)*single(eye(n-1))];
A = M*Bt;
b = -A*uMin;%约束条件向量
c = -Bt'*ydt;%目标函数
h = uMax-uMin;

%To find Feasible solution construct problem with appended slack variables
sb = single(2*(b > single(0))-1);%1添加松弛变量，-1添加剩余变量
Ai = [A diag(sb)];%新的约束矩阵
ci = single([zeros(m,1);ones(n-1,1)]);%新的目标函数
inBi = uint8(m+1:m+n-1);%索引，指向松弛或剩余变量的列
ei = int8(true(m+n-1,1));%逻辑向量
hi = [h;2*abs(b)];%新的范围向量
%Use Bounded Revised Simplex to find initial basic feasible point
[y1, inB1, e1,itlim,errsimp] = simplxuprevsol_C(Ai,ci',b,inBi,hi,ei,n-1,m+n-1,itlim);

%Check that Feasible Solution was found
if itlim<=uint16(0)
    errout = int8(-3);
    disp('Too Many Iterations Finding initial Solution');
end
if any(inB1>m)
    errout = int8(-2);
    disp('No Initial Feasible Solution found');
end
if errsimp
    errout = int8(-1);
	disp('Solver error');
end

if errout ~=int8(0)  % Construct an incorrect solution to accompany error flags
    xout = single(zeros(m,1));
    xout(inB1(1:m)) = y1(1:m);
    xout(~e1(1:m)) = -xout(~e1(1:m))+h(~e1(1:m));
    
else  % No Error continue to solve problem
    
    
    
    %Solve using initial problem from above
    [y2, inB2, e2,itlim,errsimp] = simplxuprevsol_C(A ,c',b,inB1,h,e1(1:m),n-1,m,itlim);
    
    %Construct solution to original LP problem from bounded simplex output
    %  Set non-basic variables to 0 or h based on e2
    %  Set basic variables to y2 or h-y2.
    xout = single(zeros(m,1));
    xout(inB2) = y2;
    xout(~e2) = -xout(~e2)+h(~e2);
    
    if itlim<=uint16(0)
        errout = int8(3);
        disp('Too Many Iterations Finding Final Solution');
    end
    if errsimp
        errout = int8(1);
	    disp('Solver error');
    end
end

%Transform Solution Back Into control variables
% u(i) = x(i)+umin(i) if e(i)
u = xout+uMin;
%Rescale controls so solution is not on boundary of Omega.
rho = ydt'*Bt*u/(ydt'*ydt);
if rho > 1
    u = u/rho;
end

return;
end

function [y0, inB, e,itlim,errout] = simplxuprevsol_C(A,ct,b,inB,h,e,m,n,itlim)%2，6
%  Bounded Revised Simplex
%
%function [yout, inBout,bout, itout,errout] = simplxuprevsol(A,ct,b,inB,inD,h,e,m,n,itlim)
%
%   Solves the linear program:
%          minimize c'y 
%          subject to 
%          Ay = b
%          0<= y <= h
%
%  Inputs: 
%          A [m,n]   = lhs Matrix of equaltity constraints 左手边
%          ct [1,n]  = transpose of cost vector
%          b [m,1]   = rhs vector for equality constraint 右手边
%          inB [m]   = Vector of indices of unknowns in the initial basic set
%          inD [n-m] = Vector of indices of unknowns not in the initial basic set
%          h[n,1]    = Upper Bound for unknowns
%          e[n,1]    = Sign for unknown variables (+ lower bound, - upper bound)
%  Optional inputs:
%          m,n       = number of constraints, unknowns (Opposite standard
%                      CA convention
%          itlim     = Upper bound on the allowed iterations\
%
% Outputs:
%         yout[n,1]  = Optimal output variable
%         inBout     = indices of Basic vectors in output
%         eout       = sign associate with output unknowns
%         itout      = number of iterations remaining out of itlim
%         errout     = Flag (=true) if unbounded is set
%
% Modification History
%   2002      Roger Beck  Original
%   8/2014    Roger Beck  Update for use
%   9/2014    Roger Beck  Added anti-cycling rule

%Optional Inputs  	
    	
%Tolerance for unknown == 0
tol=single(1e-8);

%Index list for non-basic variables
nind = (1:(n-m));

% Partition A
% #include <iostream>
% #include <set>
% #include <vector>
% std::vector<int> setdiff(const std::vector<int>& a, const std::vector<int>& b) {
%     std::set<int> a_set(a.begin(), a.end());
%     std::set<int> b_set(b.begin(), b.end());
%     std::vector<int> result;
%     for (int elem : a_set) {
%         if (b_set.find(elem) == b_set.end()) {
%             result.push_back(elem);
%         }
%     }
%     return result;
% }
% int main() {
%     std::vector<int> array1 = {1, 2, 3, 4, 5};
%     std::vector<int> array2 = {4, 5, 6, 7, 8};
%     std::vector<int> diff = setdiff(array1, array2);
%     for (int elem : diff) {
%         std::cout << elem << " ";
%     }
%     return 0;
% }
% inD = setdiff(uint8(1:n), inB);

tmp=uint8(1:n);
 inD = tmp(~ismember(tmp, inB));

%Adjust signs problem if variables are initialized at upper
% bounds.
A(:,~e) = -A(:,~e);
ct(~e) = -ct(~e);
b = b + A(:,~e)*h(~e);

y0 = A(:,inB)\b;  %Initial Solution

%Initialize Loop Termination Conditions
done = false;
unbounded = false;

%Main Simplex loop
while (~done  || ~unbounded ) && (itlim > uint16(0))
    itlim = itlim-uint16(1);

    %Calculate transpose of relative cost vector based on current basis
    lamt = ct(inB)/A(:,inB);
    rdt = ct(inD)-lamt*A(:,inD);
    %Find minimum relative cost
    [minr, qind] = min(rdt);
    qind=uint8(qind);
    if minr >= single(0)  % If all relative costs are positive then the solution is optimal
        done = true;
        break;
    end
    qel = inD(qind);  % Unknown to Enter the basis minimizes relative cost
    yq = A(:,inB)\A(:,qel); %Vector to enter in terms of the current Basis vector
    
    if all(abs(yq)<=tol)
      unbounded = true;
      disp(' Solution is unbounded');  % Check this condition
      break
    end

    %Compute ratio how much each current basic variable will have to move for the entering
    % variable.
    rat = y0./yq; 
    % If yq < 0 then increasing variable when it leaves the basis will minimize cost
    hinB = h(inB);
    indm = yq<single(0);
    rat(indm) = rat(indm) - hinB(indm)./yq(indm);
    % If an element yq ~=0 then it doesn't change for the entering variable and shouldn't
    %  be chosen
    indz = abs(yq)<=tol;
    rat(indz) = inf;
    % Variable to exit is moving to its minimum value
    [minrat, p] = min(rat);
   % If the minimum ratio is zero, then the solution is degenerate and the entering
   %   variable will not change the basis---invoke Bland's selection rule to avoid
   %   cycling.
    if (abs(minrat) <= tol)
       % Find negative relative cost
       indm = nind(rdt<single(0)); %Note that since minr <0 indm is not empty   
       qind = indm(1);
       qel = inD(qind);  % Unknown to Enter the basis is first indexed to avoid cycling
       yq = A(:,inB)\A(:,qel); %Vector to enter in terms of the current Basis vector
       if all(abs(yq)<=tol)
           unbounded = true;
           disp(' Solution is unbounded');  % Check this condition
           break
       end
       % Recompute rations and determine variable to leave
       rat = y0./yq; 
        % If yq < 0 then increasing variable when it leaves the basis will minimize cost
        hinB = h(inB);
        indm = yq<single(0);
        rat(indm) = rat(indm) - hinB(indm)./yq(indm);
        % If an element yq ~=0 then it doesn't change for the entering variable and shouldn't
        %  be chosen
        indz = abs(yq)<=tol;
        rat(indz) = inf;

        % Variable to exit is moving to its minimum value--Note that min returns the lowest index minimum
        [minrat, p] = min(rat);
    end

  % Maintain the bounded simplex as only having lower bounds by recasting 
  % any variable that needs to move to its opposite bound.
    if (minrat >= h(qel))
           %Case 1: Entering variable goes to opposite bound and current basis is maintained
            e(qel) = ~e(qel);
            A(:,qel) = -A(:,qel);
             b = b + A(:,qel)*h(qel);
             ct(qel) = -ct(qel);
    elseif yq(p) > single(0)
           %Case 2: Leaving variable returns to lower bound (0)	
           pel = inB(p);
           inB(p)= qel;
           inD(qind)= pel;
     else
           %Case 2: Leaving variable moves to upper bound	
            pel = inB(p);
            e(pel)=~e(pel);
            A(:,pel) = -A(:,pel);
            inB(p)= qel;
            inD(qind)= pel;
            ct(pel) = -ct(pel);
            b = b + A(:,pel)*h(pel);
     end
        
    y0 = A(:,inB)\b; % Compute new Basic solution;
end
errout = unbounded;     
end