%% Author : Zini Wang %%


clear
clf
clc
close all
PH=5;   %预测时域指定，控制时域与其相同
%(功率需求Follow_HESSPower [w],行驶速度Follow_Velocity   [m/s]
load('NYCC.mat')

% 下含Velocity_NYCC+Velocity_SC03+Velocity_US06
load('Velocity.mat')  %[600,301]，每一行为包括当前时刻共301个未来速度
Speed=Velocity_NYCC;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 电池组SOC范围[0.05,1],超级电容组SOC范围[0.5,0.9] %
SOCbat_initial = 0.95;              %  电池组初始SOC
SOCsc_initial = 0.75;                %  超级电容组初始SOC
Rsc = 0.01467;        % [Ohm] 超级电容内阻
Rbat = 0.09375;         % [Ohm] 电池内阻
Voc = 400;                          % [V] 电池组开路电压
Vmax=297;                           % [V] 超级电容组最大电压
Vsc_initial=SOCsc_initial*Vmax;
REAL_Id=Follow_HESSPower/(Voc);                % [A] 实际负载电流
EMS_distance=round(sum(Follow_Velocity),2);             % [m]工况里程
Time=length(Follow_HESSPower);                 % [s] 工况时长



% MPC控制对象初始化(包括约束条件、代价函数、状态方程等）
nlobj=MPC_initial(PH);
X0=[SOCbat_initial;Vsc_initial];
lastcontrol=0;


set(0,'defaultfigurecolor','w');
% 绘制云层预测的速度、功率需求及负载电流 %
figure(1);
subplot(2,1,1);
plot(Follow_Velocity)
title('工况速度')
xlabel('时间(s)');
ylabel('速度(m/s)');
subplot(2,1,2);
plot(Follow_HESSPower/1000)
title('实际功率需求')
xlabel('时间(s)');
ylabel('功率(kW)');
Ts=nlobj.Ts;
totalcaltime=zeros(Time,1);
 exitflag=zeros(Time,1);
totalmv=zeros(Time,1);
MVopt=zeros(PH+1,Time);
Vscopt=zeros(PH+1,Time);
SOCbatopt=zeros(PH+1,Time);


nloptions = nlmpcmoveopt;
nloptions.Parameters ={Ts};
tic
for i=1:Time
PredictV=Speed(i,1:PH+1);    %PH cann't exceed 300
Mesure_distur=vehicle_DynamicModel(PredictV);      %列向量
temp=toc;
[mv,opt,info] = nlmpcmove(nlobj,X0,lastcontrol,[],Mesure_distur,nloptions);
totalcaltime(i)=toc-temp;
fprintf("%d s求解用时 %.2f\n",i,totalcaltime(i));
% cost(i)=info.Cost;
% Iterations(i)=info.Iterations;
%totalmv(i)=mv;
MVopt(1:PH+1,i)=info.MVopt;
Vscopt(1:PH+1,i)=info.Xopt(:,2);
SOCbatopt(1:PH+1,i)=info.Xopt(:,1);
lastcontrol=info.MVopt(1,:);
X0=info.Xopt(2,:)';
end
toc
EMS_Psctotal=MVopt(1,:);
EMS_SOCbat=SOCbatopt(1,:);
EMS_Vsc=Vscopt(1,:);
EMS_Psctotal=(EMS_Psctotal*1000)';  %[W]
EMS_Vsc=EMS_Vsc';
EMS_SOCbat = EMS_SOCbat';
%计算电池组、超级电容组提供的功率
EMS_Isc=EMS_Psctotal./EMS_Vsc;
EMS_PSC= EMS_Psctotal-EMS_Isc.^2*Rsc;

DCefficiency= 0.95;
EMS_Pbat =Follow_HESSPower-EMS_PSC./DCefficiency;     
k=find(EMS_PSC>0);
EMS_Pbat(k) = Follow_HESSPower(k)-EMS_PSC(k).*DCefficiency; 
EMS_Ibat=real((Voc-sqrt(Voc^2-4*Rbat*EMS_Pbat))/(2*Rbat));

[elecost,capacitycost,Qloss_Aver] = costcalculation(EMS_PSC,EMS_Pbat,EMS_Ibat);
elecost=elecost/(EMS_distance/1000);
capacitycost=capacitycost/(EMS_distance/1000);
totalcost=(elecost+capacitycost);



h(1)=figure;
plot(EMS_SOCbat,'b');
hold on
plot(EMS_Vsc/297,'r')
legend('电池SOC轨迹','超级电容SOC轨迹');

h(2)=figure;
subplot(2,1,1)
plot(REAL_Id,'k')
grid on
xlabel('时间(s)');
ylabel('实际负载电流(A)');
legend('实际负载电流')
subplot(2,1,2)
plot(EMS_Isc,'r')
hold on;
plot(EMS_Ibat,'b')
grid on
xlabel('时间(s)');
ylabel('电流(A)');
legend('超级电容电流','电池电流')

h(3)=figure;
hold on
plot(Follow_HESSPower/1000,'b');
plot(EMS_PSC/1000,'g');
plot(EMS_Pbat/1000,'r');
xlabel('时间[s]');
ylabel('功率[kW]');
grid on
legend('工况需求功率','超级电容功率','锂电池功率')

h(4)=figure;
stem(totalcaltime);



fprintf('百公里总成本           %.3f。\n',totalcost*100);
fprintf('百公里电池容量衰减成本 %.3f。\n',capacitycost*100);
fprintf('百公里电力成本         %.3f。\n',elecost*100);
fprintf('终端SC荷电状态         %.4f。\n',X0(2)/297);
fprintf('秒/步         %.3f。\n',sum(totalcaltime)/(i-1));
% savefig(h,'normal0-3_US0630.fig','compact');
% save(['C:\Users\王子妮\Desktop\毕业设计\simulation\code\prediction_horizon_research-code\US06\','normal0-3-optVscPsc_US0630.mat'],'MVopt','Vscopt');

