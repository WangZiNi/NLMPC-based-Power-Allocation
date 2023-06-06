function nlobj=MPC_initial(PH)
%ģ��Ԥ����Ƴ�ʼ������
%%��������С����
Qbat=120;            % [Ah] ���������
Vsc_max=297;          % [V] �������ݱ�Ƶ�ѹ
%%�������������С����
% Iuc_max = 800;        % [A] Maximum SC discharge current
% Iuc_min = -800;       % [A] Maximum SC charge current
Puc_max = 200;        % [kW] �����������ŵ繦��
Puc_min = -200;       % [kW] ������������繦��
%%����״̬�����������������������
nx = 2;ny = 1;nu = 2;
nlobj = nlmpc(nx, ny,'MV',2,'MD',1);
%%MPC��������������
Ts=1;nlobj.Ts=Ts;
%%����Ԥ��ʱ��Ϳ���ʱ��
nlobj.PredictionHorizon=PH;
nlobj.ControlHorizon =PH;
%%��������
nlobj.Model.NumberOfParameters =1;
nlobj.Model.StateFcn =@(x,u,Ts) MPC_stateconvert(x,u,Ts);
%%ʹ����ɢʱ��Ԥ��ģ��
nlobj.Model.IsContinuousTime = false;
%%%���ù۲ⷽ��
nlobj.Model.OutputFcn = @(x,u,Ts) [x(2)];
%%״̬������ΧԼ��
nlobj.States(1).Min=0.05;
nlobj.States(1).Max=1;
nlobj.States(2).Min=0.5*Vsc_max;
nlobj.States(2).Max=0.9*Vsc_max;

%%���Ʊ�����ΧԼ��
nlobj.MV(1).Min=Puc_min;
nlobj.MV(1).Max=Puc_max;


%%����ƽ������-�����Ե�ʽԼ��
nlobj.Optimization.CustomIneqConFcn = @(X,U,e,data,Ts) MPC_inEquaConstrain(X,U,e,data,Ts);
%%%����MPC���ۺ���
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
% setterminal(nlobj,Y,U);  %ֻ������mpc()�����Ķ���
end

