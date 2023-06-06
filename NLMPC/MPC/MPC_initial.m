function nlobj=MPC_initial(PH)
%模型预测控制初始化函数
%%电池最大最小电流
Qbat=120;            % [Ah] 电池组容量
Vsc_max=297;          % [V] 超级电容标称电压
%%超级电容最大最小电流
% Iuc_max = 800;        % [A] Maximum SC discharge current
% Iuc_min = -800;       % [A] Maximum SC charge current
Puc_max = 200;        % [kW] 超级电容最大放电功率
Puc_min = -200;       % [kW] 超级电容最大充电功率
%%设置状态变量、输出量、控制量个数
nx = 2;ny = 1;nu = 2;
nlobj = nlmpc(nx, ny,'MV',2,'MD',1);
%%MPC控制器采样周期
Ts=1;nlobj.Ts=Ts;
%%设置预测时域和控制时域
nlobj.PredictionHorizon=PH;
nlobj.ControlHorizon =PH;
%%参数个数
nlobj.Model.NumberOfParameters =1;
nlobj.Model.StateFcn =@(x,u,Ts) MPC_stateconvert(x,u,Ts);
%%使用离散时间预测模型
nlobj.Model.IsContinuousTime = false;
%%%设置观测方程
nlobj.Model.OutputFcn = @(x,u,Ts) [x(2)];
%%状态变量范围约束
nlobj.States(1).Min=0.05;
nlobj.States(1).Max=1;
nlobj.States(2).Min=0.5*Vsc_max;
nlobj.States(2).Max=0.9*Vsc_max;

%%控制变量范围约束
nlobj.MV(1).Min=Puc_min;
nlobj.MV(1).Max=Puc_max;


%%功率平衡条件-非线性等式约束
nlobj.Optimization.CustomIneqConFcn = @(X,U,e,data,Ts) MPC_inEquaConstrain(X,U,e,data,Ts);
%%%设置MPC代价函数
nlobj.Optimization.CustomCostFcn = @(X,U,e,data,Ts) MPC_Costfunc(X,U,e,data,Ts);

nlobj.Optimization.ReplaceStandardCost=true;
nlobj.Optimization.UseSuboptimalSolution=true;
nlobj.Optimization.SolverOptions.MaxIterations=1000;
nlobj.Optimization.SolverOptions.Algorithm='sqp'; %active-set % interior-point %sqp
%nlobj.Optimization.SolverOptions.UseParallel=true;
% nlobj.Optimization.SolverOptions.OptimalityTolerance=1.0000e-6;
% nlobj.Optimization.SolverOptions.ConstraintTolerance=1.0000e-6;
% nlobj.Optimization.SolverOptions.StepTolerance=1.0000e-6;
% Y = struct('Min',0.7*Vsc_max,'Max',0.8*Vsc_max);%soft   
% U = struct('Min',-inf,'Max',inf);
% setterminal(nlobj,Y,U);  %只用于由mpc()创建的对象
end

