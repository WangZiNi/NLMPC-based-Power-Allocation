%% Author : Zini Wang %%


clear
clf
clc
close all
PH=5;   %prediction horizon
%Follow_HESSPower [w],Follow_Velocity   [m/s]
load('NYCC.mat')

% include Velocity_NYCC+Velocity_SC03+Velocity_US06
load('Velocity.mat')  %[600,301]
Speed=Velocity_NYCC;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SoC_bat [0.05,1],SoC_sc: [0.5,0.9] %
SOCbat_initial = 0.95;              
SOCsc_initial = 0.75;                
Rsc = 0.01467;        % [Ohm] resistance 
Rbat = 0.09375;         % [Ohm] resistance
Voc = 400;                          % [V] OCV of batteries
Vmax=297;                           % [V] maximum voltage of SCs
Vsc_initial=SOCsc_initial*Vmax;
REAL_Id=Follow_HESSPower/(Voc);                % [A] load current
EMS_distance=round(sum(Follow_Velocity),2);             % [m] total distance
Time=length(Follow_HESSPower);                 % [s] 



% initialize the MPC object
nlobj=MPC_initial(PH);
X0=[SOCbat_initial;Vsc_initial];
lastcontrol=0;


set(0,'defaultfigurecolor','w');
figure(1);
subplot(2,1,1);
plot(Follow_Velocity)
title('Velocity')
xlabel('times(s)');
ylabel('Velocity(m/s)');
subplot(2,1,2);
plot(Follow_HESSPower/1000)
title('Power demand')
xlabel('times(s)');
ylabel('Power(kW)');
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
Mesure_distur=vehicle_DynamicModel(PredictV);      
temp=toc;
[mv,opt,info] = nlmpcmove(nlobj,X0,lastcontrol,[],Mesure_distur,nloptions);
totalcaltime(i)=toc-temp;
fprintf("at %d s time used: %.2f\n",i,totalcaltime(i));
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
%Psc/Pbat calculation
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
legend('SoCbat','SoCsc');

h(2)=figure;
subplot(2,1,1)
plot(REAL_Id,'k')
grid on
xlabel('times(s)');
ylabel('current demand(A)');
legend('current demand')
subplot(2,1,2)
plot(EMS_Isc,'r')
hold on;
plot(EMS_Ibat,'b')
grid on
xlabel('times(s)');
ylabel('current(A)');
legend('Isc','Ibat')

h(3)=figure;
hold on
plot(Follow_HESSPower/1000,'b');
plot(EMS_PSC/1000,'g');
plot(EMS_Pbat/1000,'r');
xlabel('times[s]');
ylabel('Power[kW]');
grid on
legend('demand','Psc','Pbat')

h(4)=figure;
stem(totalcaltime);



fprintf('total cost per 100km           %.3f。\n',totalcost*100);
fprintf('battery degradation cost per 100km %.3f。\n',capacitycost*100);
fprintf('electricity cost per 100km         %.3f。\n',elecost*100);
fprintf('terminal SoCsc         %.4f。\n',X0(2)/297);
fprintf('calculation time used: s per step         %.3f。\n',sum(totalcaltime)/(i-1));
% savefig(h,'normal0-3_US0630.fig','compact');
% save(['normal0-3-optVscPsc_US0630.mat'],'MVopt','Vscopt');

