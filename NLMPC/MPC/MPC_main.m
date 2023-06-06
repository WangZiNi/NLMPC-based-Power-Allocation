%% Author : Zini Wang %%


clear
clf
clc
close all
PH=5;   %Ԥ��ʱ��ָ��������ʱ��������ͬ
%(��������Follow_HESSPower [w],��ʻ�ٶ�Follow_Velocity   [m/s]
load('NYCC.mat')

% �º�Velocity_NYCC+Velocity_SC03+Velocity_US06
load('Velocity.mat')  %[600,301]��ÿһ��Ϊ������ǰʱ�̹�301��δ���ٶ�
Speed=Velocity_NYCC;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �����SOC��Χ[0.05,1],����������SOC��Χ[0.5,0.9] %
SOCbat_initial = 0.95;              %  ������ʼSOC
SOCsc_initial = 0.75;                %  �����������ʼSOC
Rsc = 0.01467;        % [Ohm] ������������
Rbat = 0.09375;         % [Ohm] �������
Voc = 400;                          % [V] ����鿪·��ѹ
Vmax=297;                           % [V] ��������������ѹ
Vsc_initial=SOCsc_initial*Vmax;
REAL_Id=Follow_HESSPower/(Voc);                % [A] ʵ�ʸ��ص���
EMS_distance=round(sum(Follow_Velocity),2);             % [m]�������
Time=length(Follow_HESSPower);                 % [s] ����ʱ��



% MPC���ƶ����ʼ��(����Լ�����������ۺ�����״̬���̵ȣ�
nlobj=MPC_initial(PH);
X0=[SOCbat_initial;Vsc_initial];
lastcontrol=0;


set(0,'defaultfigurecolor','w');
% �����Ʋ�Ԥ����ٶȡ��������󼰸��ص��� %
figure(1);
subplot(2,1,1);
plot(Follow_Velocity)
title('�����ٶ�')
xlabel('ʱ��(s)');
ylabel('�ٶ�(m/s)');
subplot(2,1,2);
plot(Follow_HESSPower/1000)
title('ʵ�ʹ�������')
xlabel('ʱ��(s)');
ylabel('����(kW)');
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
Mesure_distur=vehicle_DynamicModel(PredictV);      %������
temp=toc;
[mv,opt,info] = nlmpcmove(nlobj,X0,lastcontrol,[],Mesure_distur,nloptions);
totalcaltime(i)=toc-temp;
fprintf("%d s�����ʱ %.2f\n",i,totalcaltime(i));
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
%�������顢�����������ṩ�Ĺ���
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
legend('���SOC�켣','��������SOC�켣');

h(2)=figure;
subplot(2,1,1)
plot(REAL_Id,'k')
grid on
xlabel('ʱ��(s)');
ylabel('ʵ�ʸ��ص���(A)');
legend('ʵ�ʸ��ص���')
subplot(2,1,2)
plot(EMS_Isc,'r')
hold on;
plot(EMS_Ibat,'b')
grid on
xlabel('ʱ��(s)');
ylabel('����(A)');
legend('�������ݵ���','��ص���')

h(3)=figure;
hold on
plot(Follow_HESSPower/1000,'b');
plot(EMS_PSC/1000,'g');
plot(EMS_Pbat/1000,'r');
xlabel('ʱ��[s]');
ylabel('����[kW]');
grid on
legend('����������','�������ݹ���','﮵�ع���')

h(4)=figure;
stem(totalcaltime);



fprintf('�ٹ����ܳɱ�           %.3f��\n',totalcost*100);
fprintf('�ٹ���������˥���ɱ� %.3f��\n',capacitycost*100);
fprintf('�ٹ�������ɱ�         %.3f��\n',elecost*100);
fprintf('�ն�SC�ɵ�״̬         %.4f��\n',X0(2)/297);
fprintf('��/��         %.3f��\n',sum(totalcaltime)/(i-1));
% savefig(h,'normal0-3_US0630.fig','compact');
% save(['C:\Users\������\Desktop\��ҵ���\simulation\code\prediction_horizon_research-code\US06\','normal0-3-optVscPsc_US0630.mat'],'MVopt','Vscopt');

