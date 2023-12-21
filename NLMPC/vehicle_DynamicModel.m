function power = vehicle_DynamicModel(Velocity)
%vehicle dynamic model
%transfer vehicle velocity to power demand
N=length(Velocity);
%parameters
elta_md=0.85;
elta_T=0.9;
elta_r=0.65;
m=1460;   %1460+660  kg
g=9.8;
Cd=0.28;
Af=2.2;
p=1.29;
%%rolling resistance coefficient
f=0.016;
calP=m*g*f*Velocity(1:N-1)+0.5*Cd*Af*p*(Velocity(1:N-1)).^3+m*Velocity(1:N-1).*(Velocity(2:N)-Velocity(1:N-1));
power=calP/(elta_md*elta_T);
k=find(calP<0);
power(k)=calP(k)*elta_r;
power=reshape(power,N-1,1);
end

