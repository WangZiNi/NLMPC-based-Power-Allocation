function [elecost,capacitycost,Qloss_Aver] = costcalculation(Pscout,Pbatout,Ibat)
%电成本、电池老化成本的计算函数
N=length(Pscout);
QB = 120;            % [Ah] 电池容量
Voc = 400;             % [V] 电池组开路电压

% 电池老化模型参数
Price_bat = 1050;       % [RMB/kWh] 电池价格
Price_ele = 0.6;        % [RMB/kWh] 用电价格
Degrad_bat = 0.2;       % 电池容量衰减限制
T_bat = 273+25;         % [K] 绝对温度
Ea = 15162;             % [J] 活化能 from Arrhenius law
B = 1516;               %  C_rate补偿因子
R = 8.314;              % [J/(mol*K)] 气体常数
A = 0.0032;             % 指数因子
z = 0.824;              % 时间因子
Ts = 1;                  % [s] 采样周期
% Rsc = 0.01467;        % [Ohm] 超级电容内阻
% Rbat = 0.09375;         % [Ohm] 电池内阻
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
for i=0:0.05:0.2    % 初始SoH循环，从100%~80%，步长5%，共5种情况
    for k=1:N    % 时间循环，计算每种初始SoH下的Qloss        
        %SoH不同时，Crate不同
        Crate(k) = abs(Ibat(k))/(QB*(1-i));
        % Qloss 绝对值   以1为基准，假设已经损耗了万分之一（零），百分之五，百分之十，百分之十五，百分之二十（需要换电）
        if k==1
            Qloss(k)=0.0001+i;  
        else
            deltaQloss(k)=Ah(k) * z*A^(1/z) * exp((-Ea+B*Crate(k))/(z*R*T_bat)) * Qloss(k-1)^((z-1)/z);
            Qloss(k) = Qloss(k-1) + deltaQloss(k);    
        end    
        %记录5种不同SoH情况的总Qloss绝对值
        Qloss_Sum(k) = Qloss_Sum(k) + Qloss(k);
    end
end
%计算平均值
Qloss_Aver=Qloss_Sum/5;
capacitycost=((QB*Voc*Price_bat)/(Degrad_bat*1000)) * (Qloss_Aver(end)-Qloss_Aver(1));
elecost=sum(Pscout+Pbatout)*Ts/(3600*1000)*Price_ele;
end

