function [elecost,capacitycost,Qloss_Aver] = costcalculation(Pscout,Pbatout,Ibat)
%total cost calculation function
N=length(Pscout);
QB = 120;            % [Ah] capacity of batteries
Voc = 400;             % [V]OCV　of batteries

% battery degradation parameters
Price_bat = 1050;       % [RMB/kWh] battery price
Price_ele = 0.6;        % [RMB/kWh] electricity price
Degrad_bat = 0.2;       % battery degradation limit
T_bat = 273+25;         % [K] 
Ea = 15162;             % [J] 
B = 1516;               % 
R = 8.314;              % [J/(mol*K)] 
A = 0.0032;             % 
z = 0.824;              % 
Ts = 1;                  % [s] control interval
% Rsc = 0.01467;        % [Ohm] resistance
% Rbat = 0.09375;         % [Ohm] resistance
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
for i=0:0.05:0.2    % consider different initial degradation level
    for k=1:N       %      
        %
        Crate(k) = abs(Ibat(k))/(QB*(1-i));
        % Qloss
        if k==1
            Qloss(k)=0.0001+i;  
        else
            deltaQloss(k)=Ah(k) * z*A^(1/z) * exp((-Ea+B*Crate(k))/(z*R*T_bat)) * Qloss(k-1)^((z-1)/z);
            Qloss(k) = Qloss(k-1) + deltaQloss(k);    
        end    
        %
        Qloss_Sum(k) = Qloss_Sum(k) + Qloss(k);
    end
end
%average value of Qloss at 5 different degradation levels
Qloss_Aver=Qloss_Sum/5;
capacitycost=((QB*Voc*Price_bat)/(Degrad_bat*1000)) * (Qloss_Aver(end)-Qloss_Aver(1));
elecost=sum(Pscout+Pbatout)*Ts/(3600*1000)*Price_ele;
end

