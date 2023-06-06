function [elecost,capacitycost,Qloss_Aver] = costcalculation(Pscout,Pbatout,Ibat)
%��ɱ�������ϻ��ɱ��ļ��㺯��
N=length(Pscout);
QB = 120;            % [Ah] �������
Voc = 400;             % [V] ����鿪·��ѹ

% ����ϻ�ģ�Ͳ���
Price_bat = 1050;       % [RMB/kWh] ��ؼ۸�
Price_ele = 0.6;        % [RMB/kWh] �õ�۸�
Degrad_bat = 0.2;       % �������˥������
T_bat = 273+25;         % [K] �����¶�
Ea = 15162;             % [J] ��� from Arrhenius law
B = 1516;               %  C_rate��������
R = 8.314;              % [J/(mol*K)] ���峣��
A = 0.0032;             % ָ������
z = 0.824;              % ʱ������
Ts = 1;                  % [s] ��������
% Rsc = 0.01467;        % [Ohm] ������������
% Rbat = 0.09375;         % [Ohm] �������
Qloss_Aver=zeros(1,N);


% for i=1:N
% elecost(i)= Price_ele*(Pscout(i)+Pbatout(i))*Ts/(3600*1000);
% Ah = abs(Ibat(i))*Ts/(3600);
% Crate = abs(Ibat(i))/(QB);
% deltaQloss=Ah .* 7.73e-4 .* exp((-Ea+B*Crate)/(z*R*T_bat)) * (0.05)^(-0.2136);
% capacitycost(i)=((Voc*QB*Price_bat)/1000)*((deltaQloss)/Degrad_bat);
% end
% for  i=2:N
%     elecost(i)=elecost(i)+elecost(i-1);
%     capacitycost(i)= capacitycost(i)+ capacitycost(i-1);
% end

    Ah= abs(Ibat)*Ts/3600;
    Qloss_Sum = zeros(1,N);
for i=0:0.05:0.2    % ��ʼSoHѭ������100%~80%������5%����5�����
    for k=1:N    % ʱ��ѭ��������ÿ�ֳ�ʼSoH�µ�Qloss        
        %SoH��ͬʱ��Crate��ͬ
        Crate(k) = abs(Ibat(k))/(QB*(1-i));
        % Qloss ����ֵ   ��1Ϊ��׼�������Ѿ���������֮һ���㣩���ٷ�֮�壬�ٷ�֮ʮ���ٷ�֮ʮ�壬�ٷ�֮��ʮ����Ҫ���磩
        if k==1
            Qloss(k)=0.0001+i;  
        else
            deltaQloss(k)=Ah(k) * z*A^(1/z) * exp((-Ea+B*Crate(k))/(z*R*T_bat)) * Qloss(k-1)^((z-1)/z);
            Qloss(k) = Qloss(k-1) + deltaQloss(k);    
        end    
        %��¼5�ֲ�ͬSoH�������Qloss����ֵ
        Qloss_Sum(k) = Qloss_Sum(k) + Qloss(k);
    end
end
%����ƽ��ֵ
Qloss_Aver=Qloss_Sum/5;
capacitycost=((QB*Voc*Price_bat)/(Degrad_bat*1000)) * (Qloss_Aver(end)-Qloss_Aver(1));
elecost=sum(Pscout+Pbatout)*Ts/(3600*1000)*Price_ele;
end

