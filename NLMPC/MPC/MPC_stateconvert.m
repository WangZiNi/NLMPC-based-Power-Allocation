function Xk1 = MPC_stateconvert(x,u,Ts)
%states vector: [SoCbat,Vsc]'
%control variable: total output power of SCs
%u(1) MD：Pdemand，
%u(2) P_SC
Csc=76.36;
%Vmax=297;
QB=120;
Voc=400;
Rsc = 0.01467;        % [Ohm] resistance
Rbat = 0.09375;         % [Ohm] resistance
Isc0=u(2)*1000/x(2);

Pscout0= u(2)*1000-(Isc0)^2*Rsc;

%% define DC/DC efficiency as a function of Vsc and Isc
% Vsc_vector =linspace(130,285,20);
% Isc_vector =[-800.00 	-789.87 	-779.75 	-769.62 	-759.49 	-749.37 	-739.24 	-729.11 	-718.99 	-708.86 	-698.74 	-688.61 	-678.48 	-668.36 	-658.23 	-648.10 	-637.98 	-627.85 	-617.72 	-607.60 	-597.47 	-587.34 	-577.22 	-567.09 	-556.97 	-546.84 	-536.71 	-526.59 	-516.46 	-506.33 	-496.21 	-486.08 	-475.95 	-465.83 	-455.70 	-445.57 	-435.45 	-425.32 	-415.19 	-405.07 	-394.94 	-384.82 	-374.69 	-364.56 	-354.44 	-344.31 	-334.18 	-324.06 	-313.93 	-303.80 	-293.68 	-283.55 	-273.42 	-263.30 	-253.17 	-243.04 	-232.92 	-222.79 	-212.67 	-202.54 	-192.41 	-182.29 	-172.16 	-162.03 	-151.91 	-141.78 	-131.65 	-121.53 	-111.40 	-101.27 	-91.15 	-81.02 	-70.90 	-60.77 	-50.64 	-40.52 	-30.39 	-20.26 	-10.14 	0 	10.14 	20.26 	30.39 	40.52 	50.64 	60.77 	70.90 	81.02 	91.15 	101.27 	111.40 	121.53 	131.65 	141.78 	151.91 	162.03 	172.16 	182.29 	192.41 	202.54 	212.67 	222.79 	232.92 	243.04 	253.17 	263.30 	273.42 	283.55 	293.68 	303.80 	313.93 	324.06 	334.18 	344.31 	354.44 	364.56 	374.69 	384.82 	394.94 	405.07 	415.19 	425.32 	435.45 	445.57 	455.70 	465.83 	475.95 	486.08 	496.21 	506.33 	516.46 	526.59 	536.71 	546.84 	556.97 	567.09 	577.22 	587.34 	597.47 	607.60 	617.72 	627.85 	637.98 	648.10 	658.23 	668.36 	678.48 	688.61 	698.74 	708.86 	718.99 	729.11 	739.24 	749.37 	759.49 	769.62 	779.75 	789.87 	800.00];           
% eta_dcdc = [0.77 	0.77 	0.78 	0.78 	0.78 	0.78 	0.78 	0.79 	0.79 	0.79 	0.79 	0.80 	0.80 	0.80 	0.80 	0.80 	0.81 	0.81 	0.81 	0.81 	0.82 	0.82 	0.82 	0.82 	0.83 	0.83 	0.83 	0.83 	0.83 	0.84 	0.84 	0.84 	0.84 	0.85 	0.85 	0.85 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.95 	 	0.90 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.95 	0.95 	0.94 	0.94 	0.94 	0.93 	0.93 	0.92 	0.92 	0.92 	0.91 	0.91 	0.90 	0.90 	0.90 	0.89 	0.89 	0.88 	0.88 	0.87 	0.87 	0.87 	0.86 	0.86 	0.85 	0.85 	0.84 	0.84 	0.83 	0.83 	0.83 	0.82 	0.82 	0.81 	0.81 	0.80 	0.80 	0.79 	0.79 	0.79 	0.78 	0.78 	0.77 	0.77 	0.76 	0.76 	0.75 	0.75 	0.74 	0.74 	0.73 	0.73 	0.72 	0.72 	0.72 	0.71 	0.71 	0.70 	0.70 	0.69 	0.69 	0.68 	0.68 	0.67 	0.67 	0.66 	0.66 	0.65 	0.65 	0.64 
%        0.78 	0.78 	0.79 	0.79 	0.79 	0.79 	0.79 	0.80 	0.80 	0.80 	0.80 	0.80 	0.81 	0.81 	0.81 	0.81 	0.82 	0.82 	0.82 	0.82 	0.82 	0.83 	0.83 	0.83 	0.83 	0.84 	0.84 	0.84 	0.84 	0.84 	0.85 	0.85 	0.85 	0.85 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.95 	 	0.90 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.95 	0.95 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.92 	0.92 	0.91 	0.91 	0.91 	0.90 	0.90 	0.89 	0.89 	0.89 	0.88 	0.88 	0.87 	0.87 	0.87 	0.86 	0.86 	0.85 	0.85 	0.84 	0.84 	0.84 	0.83 	0.83 	0.82 	0.82 	0.82 	0.81 	0.81 	0.80 	0.80 	0.79 	0.79 	0.79 	0.78 	0.78 	0.77 	0.77 	0.76 	0.76 	0.76 	0.75 	0.75 	0.74 	0.74 	0.73 	0.73 	0.72 	0.72 	0.72 	0.71 	0.71 	0.70 	0.70 	0.69 	0.69 	0.68 	0.68 	0.68 	0.67 	0.67 
%         0.79 	0.79 	0.79 	0.80 	0.80 	0.80 	0.80 	0.80 	0.81 	0.81 	0.81 	0.81 	0.81 	0.82 	0.82 	0.82 	0.82 	0.83 	0.83 	0.83 	0.83 	0.83 	0.84 	0.84 	0.84 	0.84 	0.84 	0.85 	0.85 	0.85 	0.85 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.97 	0.96 	 	0.90 	0.96 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.92 	0.92 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.89 	0.89 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.86 	0.86 	0.85 	0.85 	0.85 	0.84 	0.84 	0.83 	0.83 	0.83 	0.82 	0.82 	0.81 	0.81 	0.81 	0.80 	0.80 	0.79 	0.79 	0.79 	0.78 	0.78 	0.77 	0.77 	0.77 	0.76 	0.76 	0.75 	0.75 	0.75 	0.74 	0.74 	0.73 	0.73 	0.72 	0.72 	0.72 	0.71 	0.71 	0.70 	0.70 	0.70 	0.69 	0.69 
%         0.80 	0.80 	0.80 	0.80 	0.81 	0.81 	0.81 	0.81 	0.81 	0.82 	0.82 	0.82 	0.82 	0.82 	0.83 	0.83 	0.83 	0.83 	0.83 	0.84 	0.84 	0.84 	0.84 	0.84 	0.85 	0.85 	0.85 	0.85 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.97 	0.97 	0.96 	 	0.90 	0.97 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.90 	0.90 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.86 	0.86 	0.85 	0.85 	0.85 	0.84 	0.84 	0.84 	0.83 	0.83 	0.82 	0.82 	0.82 	0.81 	0.81 	0.81 	0.80 	0.80 	0.79 	0.79 	0.79 	0.78 	0.78 	0.78 	0.77 	0.77 	0.76 	0.76 	0.76 	0.75 	0.75 	0.74 	0.74 	0.74 	0.73 	0.73 	0.72 	0.72 	0.72 	0.71 	0.71 	0.70 
%         0.81 	0.81 	0.81 	0.81 	0.81 	0.82 	0.82 	0.82 	0.82 	0.82 	0.82 	0.83 	0.83 	0.83 	0.83 	0.83 	0.84 	0.84 	0.84 	0.84 	0.84 	0.85 	0.85 	0.85 	0.85 	0.86 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.97 	0.96 	 	0.90 	0.97 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.85 	0.85 	0.84 	0.84 	0.84 	0.83 	0.83 	0.83 	0.82 	0.82 	0.82 	0.81 	0.81 	0.81 	0.80 	0.80 	0.79 	0.79 	0.79 	0.78 	0.78 	0.78 	0.77 	0.77 	0.77 	0.76 	0.76 	0.75 	0.75 	0.75 	0.74 	0.74 	0.74 	0.73 	0.73 	0.72 	0.72 
%         0.81 	0.81 	0.82 	0.82 	0.82 	0.82 	0.82 	0.83 	0.83 	0.83 	0.83 	0.83 	0.84 	0.84 	0.84 	0.84 	0.84 	0.84 	0.85 	0.85 	0.85 	0.85 	0.85 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.96 	 	0.90 	0.97 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.84 	0.84 	0.84 	0.83 	0.83 	0.83 	0.82 	0.82 	0.82 	0.81 	0.81 	0.81 	0.80 	0.80 	0.79 	0.79 	0.79 	0.78 	0.78 	0.78 	0.77 	0.77 	0.77 	0.76 	0.76 	0.76 	0.75 	0.75 	0.75 	0.74 	0.74 	0.74 
%         0.82 	0.82 	0.82 	0.82 	0.83 	0.83 	0.83 	0.83 	0.83 	0.84 	0.84 	0.84 	0.84 	0.84 	0.84 	0.85 	0.85 	0.85 	0.85 	0.85 	0.86 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.97 	 	0.90 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.84 	0.84 	0.84 	0.83 	0.83 	0.83 	0.82 	0.82 	0.82 	0.81 	0.81 	0.81 	0.80 	0.80 	0.80 	0.80 	0.79 	0.79 	0.79 	0.78 	0.78 	0.78 	0.77 	0.77 	0.77 	0.76 	0.76 	0.76 	0.75 	0.75 
%         0.83 	0.83 	0.83 	0.83 	0.83 	0.83 	0.84 	0.84 	0.84 	0.84 	0.84 	0.84 	0.85 	0.85 	0.85 	0.85 	0.85 	0.86 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	 	0.90 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.84 	0.84 	0.84 	0.84 	0.83 	0.83 	0.83 	0.82 	0.82 	0.82 	0.81 	0.81 	0.81 	0.80 	0.80 	0.80 	0.80 	0.79 	0.79 	0.79 	0.78 	0.78 	0.78 	0.77 	0.77 	0.77 	0.76 	0.76 
%         0.83 	0.83 	0.83 	0.84 	0.84 	0.84 	0.84 	0.84 	0.84 	0.85 	0.85 	0.85 	0.85 	0.85 	0.86 	0.86 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	 	0.90 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.85 	0.84 	0.84 	0.84 	0.83 	0.83 	0.83 	0.83 	0.82 	0.82 	0.82 	0.81 	0.81 	0.81 	0.80 	0.80 	0.80 	0.80 	0.79 	0.79 	0.79 	0.78 	0.78 	0.78 	0.77 	0.77 
%         0.84 	0.84 	0.84 	0.84 	0.84 	0.84 	0.85 	0.85 	0.85 	0.85 	0.85 	0.85 	0.86 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	 	0.90 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.84 	0.84 	0.84 	0.84 	0.83 	0.83 	0.83 	0.82 	0.82 	0.82 	0.82 	0.81 	0.81 	0.81 	0.80 	0.80 	0.80 	0.80 	0.79 	0.79 	0.79 	0.78 	0.78 
%         0.84 	0.84 	0.84 	0.85 	0.85 	0.85 	0.85 	0.85 	0.85 	0.86 	0.86 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	 	0.90 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.84 	0.84 	0.84 	0.84 	0.83 	0.83 	0.83 	0.83 	0.82 	0.82 	0.82 	0.81 	0.81 	0.81 	0.81 	0.80 	0.80 	0.80 	0.80 	0.79 	0.79 
%         0.85 	0.85 	0.85 	0.85 	0.85 	0.85 	0.86 	0.86 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	 	0.90 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.85 	0.84 	0.84 	0.84 	0.84 	0.83 	0.83 	0.83 	0.82 	0.82 	0.82 	0.82 	0.81 	0.81 	0.81 	0.81 	0.80 	0.80 	0.80 
%         0.85 	0.85 	0.85 	0.85 	0.86 	0.86 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	 	0.90 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.85 	0.84 	0.84 	0.84 	0.84 	0.83 	0.83 	0.83 	0.83 	0.82 	0.82 	0.82 	0.82 	0.81 	0.81 	0.81 	0.81 
%         0.85 	0.86 	0.86 	0.86 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	 	0.90 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.85 	0.85 	0.84 	0.84 	0.84 	0.84 	0.83 	0.83 	0.83 	0.83 	0.82 	0.82 	0.82 	0.82 	0.81 
%         0.86 	0.86 	0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 		0.90 	0.98 	0.98 	0.99 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.85 	0.84 	0.84 	0.84 	0.84 	0.83 	0.83 	0.83 	0.83 	0.82 	0.82 	0.82 
%         0.86 	0.86 	0.86 	0.87 	0.87 	0.87 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	 	0.90 	0.98 	0.98 	0.99 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.85 	0.84 	0.84 	0.84 	0.84 	0.84 	0.83 	0.83 	0.83 	0.83 
%         0.87 	0.87 	0.87 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.99 	0.99 	0.98 	0.98 		0.90 	0.98 	0.99 	0.99 	0.99 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.85 	0.85 	0.84 	0.84 	0.84 	0.84 	0.83 	0.83 
%         0.87 	0.87 	0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.99 	0.99 	0.98 	0.98 	 	0.90 	0.98 	0.99 	0.99 	0.99 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.85 	0.85 	0.84 	0.84 	0.84 	0.84 
%         0.87 	0.87 	0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.99 	0.99 	0.99 	0.99 	0.98 	 	0.90 	0.98 	0.99 	0.99 	0.99 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.85 	0.85 	0.84 	0.84 
%         0.87 	0.88 	0.88 	0.88 	0.88 	0.88 	0.88 	0.88 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.89 	0.90 	0.90 	0.90 	0.90 	0.90 	0.90 	0.90 	0.91 	0.91 	0.91 	0.91 	0.91 	0.91 	0.91 	0.92 	0.92 	0.92 	0.92 	0.92 	0.92 	0.92 	0.93 	0.93 	0.93 	0.93 	0.93 	0.93 	0.94 	0.94 	0.94 	0.94 	0.94 	0.94 	0.94 	0.95 	0.95 	0.95 	0.95 	0.95 	0.95 	0.96 	0.96 	0.96 	0.96 	0.96 	0.96 	0.97 	0.97 	0.97 	0.97 	0.97 	0.97 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.99 	0.99 	0.99 	0.99 	0.98 	 	0.90 	0.98 	0.99 	0.99 	0.99 	0.99 	0.98 	0.98 	0.98 	0.98 	0.98 	0.98 	0.97 	0.97 	0.97 	0.97 	0.97 	0.97 	0.96 	0.96 	0.96 	0.96 	0.96 	0.95 	0.95 	0.95 	0.95 	0.95 	0.95 	0.94 	0.94 	0.94 	0.94 	0.94 	0.93 	0.93 	0.93 	0.93 	0.93 	0.92 	0.92 	0.92 	0.92 	0.92 	0.92 	0.91 	0.91 	0.91 	0.91 	0.91 	0.90 	0.90 	0.90 	0.90 	0.90 	0.89 	0.89 	0.89 	0.89 	0.89 	0.88 	0.88 	0.88 	0.88 	0.88 	0.88 	0.87 	0.87 	0.87 	0.87 	0.87 	0.86 	0.86 	0.86 	0.86 	0.86 	0.85 	0.85 	0.85 	0.85 ];
% 
% F = griddedInterpolant({Vsc_vector,Isc_vector'}, eta_dcdc);
% efficiency0=F({x(2)',Isc0});

%% use constant DC/DC efficiency
 efficiency0=0.95;
if Pscout0<= 0
Pbatout0 = u(1)-Pscout0./efficiency0;      % Ibat grid
else
Pbatout0 = u(1)-Pscout0.*efficiency0;      % Ibat grid
end
Ibat0=real((Voc-sqrt(Voc^2-4*Rbat* Pbatout0))/(2*Rbat));
% Ibat0=Pbatout0/Voc;
Xk1(1,1)=x(1)-Ts*Ibat0/(3600*QB);
%Xk1(2)=x(2)-(Ts*Isc0)/(Csc);
Xk1(2,1) = real(sqrt( x(2)^2 - (Ts*u(2)*1000/(Csc*0.5))));  
end

