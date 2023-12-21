function nlobj=MPC_initial(PH)
%initialization function of MPC object

Qbat=120;            % [Ah] capacity of batteries
Vsc_max=297;          % [V] nominal voltage of SCs

% Iuc_max = 800;        % [A] Maximum SCs discharge current
% Iuc_min = -800;       % [A] Maximum SCs charge current
Puc_max = 200;        % [kW] Maximum SCs discharge power
Puc_min = -200;       % [kW] Maximum SCs charge power

%%number of states, outputs, and control variables
nx = 2;ny = 1;nu = 2;
nlobj = nlmpc(nx, ny,'MV',2,'MD',1);
%% control interval
Ts=1;nlobj.Ts=Ts;

nlobj.PredictionHorizon=PH;
nlobj.ControlHorizon =PH;
%% parameters setting
nlobj.Model.NumberOfParameters =1;
nlobj.Model.StateFcn =@(x,u,Ts) MPC_stateconvert(x,u,Ts);
%% continuous/discrete model
nlobj.Model.IsContinuousTime = false;
%%% output function setting
nlobj.Model.OutputFcn = @(x,u,Ts) [x(2)];
%% bounds of states
nlobj.States(1).Min=0.05;
nlobj.States(1).Max=1;
nlobj.States(2).Min=0.5*Vsc_max;
nlobj.States(2).Max=0.9*Vsc_max;

%% bounds of control moves
nlobj.MV(1).Min=Puc_min;
nlobj.MV(1).Max=Puc_max;


%% power balance conditon-nonlinear equation constraint
nlobj.Optimization.CustomIneqConFcn = @(X,U,e,data,Ts) MPC_inEquaConstrain(X,U,e,data,Ts);
%%% cost function setting
nlobj.Optimization.CustomCostFcn = @(X,U,e,data,Ts) MPC_Costfunc(X,U,e,data,Ts);

nlobj.Optimization.ReplaceStandardCost=true;
nlobj.Optimization.UseSuboptimalSolution=true;
nlobj.Optimization.SolverOptions.MaxIterations=1000;
nlobj.Optimization.SolverOptions.Algorithm='sqp'; %active-set % interior-point %sqp
%nlobj.Optimization.SolverOptions.UseParallel=true;
% nlobj.Optimization.SolverOptions.OptimalityTolerance=1.0000e-6;
% nlobj.Optimization.SolverOptions.ConstraintTolerance=1.0000e-6;
% nlobj.Optimization.SolverOptions.StepTolerance=1.0000e-6;
% Y = struct('Min',0.7*Vsc_max,'Max',0.8*Vsc_max); % soft   
% U = struct('Min',-inf,'Max',inf);
% setterminal(nlobj,Y,U);  %only for objects constructed by mpc()
end

