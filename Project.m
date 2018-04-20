<<<<<<< HEAD
clc
close all
clear vars

want_plot = 1;

%% Define Parameters of the System
L1 = 0.120;
L2 = 0.130;
I1 = 2.2e-4;
I2 = 2.5e-3;
m1 = 0.22;
m2 = 0.085;
g = 9.81;

N = 1;
Ke = 1.66e-2;
Kt = 27.4e-3;
Rm = 12;

a = m1*L1^2 + m2*L2^2 + I1 ;
b =(m1*L1 + m2*L2) * g ;
a21= b/a ;
a24=(Kt*Ke*N^2)/ (a*Rm) ;
a41= - b/a ;
a44=((a+I2)/(a*I2))*((Kt*Ke*N^2)/ (Rm));
b2 = -(Kt*N)/ (a*Rm);
b4 = ((a+I2)/(a*I2))* ((Kt*N)/ (a*Rm));

%% State Space Model
A = [0    1      0;
	a21  0      a24;
	a41  0      a44];

B = [0;
	b2;
	b4];

C = [1 0 0];

D = [];
%% Open Loop System
sys_ol = ss(A, B, C, D);

%% Closed Loop System
p = [-0.8+0.83i -0.8-0.83i  -8];

k = acker(A,B,p);

sys_cl = ss(A-B*k, B, C, D);

% %% Steady state error using DC-gain
% G = -1/(C*inv(A-B*k)*B);
%
% sys_DCgain = ss(A-B*k, B*G, C, D);

%% Steady state error using integrator
A_hat = [A [0;0;0];
	-C 0];
B_hat = [B;0];

p1 = [p p(3)-1];
k1 = acker(A_hat,B_hat,p1);

ki = k1(4);

A_new = [A-B*k1(1:3) -B*ki;
	-C 0];
B_new = [0;0;0;1];
C_new = [C 0];

sys_integrator = ss(A_new, B_new, C_new, D);
%% Observer Design

p_l = p1(1:3)*10;

L = acker(A', C', p_l)';

A_observer = [A     -B*k;
	L*C     A-B*k-L*C];
B_observer = [B; B];
C_observer = [[20 1 0] [0 0 0]];

%% Oberserver with State Space Feedback
sys_obv = ss(A_observer, B_observer, C_observer,D);

%% Observer with steady state implemented with integral

A_obv_new = [A        -B*ki-1        -B*k1(1:3);
	-C        0           zeros(1,3);
	L*C       -B*ki        A-B*k1(1:3)-L*C];
B_obv_new = [zeros(3,1); 1; zeros(3,1)];
C_obv_new = [C zeros(1,4)];

sys_obv_steady = ss(A_obv_new, B_obv_new, C_obv_new,D);

%% plot
dt = 0.001;
t = 0:dt:15;
u = vertcat([5],zeros(7500,1), [5],zeros(2000,1), [10], zeros(5498,1));

y_CL = lsim(sys_cl, u, t);
y_Obv_SS = lsim(sys_obv_steady, u, t);

figure(1)
subplot(2,1,1)
plot (t,y_CL,'k','LineWidth',2)
hold on
plot (t, u,'r', 'LineWidth', 2)
grid on
mylegend=legend ('System Response', 'Input')
set (mylegend,'FontSize',10,'Location','SouthEast')
myxlabel=xlabel ('time [s]')
myylabel=ylabel ('Displacement [m]')
set (myxlabel,'FontSize',12)
set (myylabel,'FontSize',12)
title('Closed Loop')

subplot(2,1,2)
plot (t, y_Obv_SS,'k', 'LineWidth', 2)
hold on
plot (t, u,'r', 'LineWidth', 2)
grid on
mylegend=legend ('System Reponse', 'Input')
set (mylegend,'FontSize',10,'Location','SouthEast')
myxlabel=xlabel ('time [s]')
myylabel=ylabel ('Displacement [m]')
set (myxlabel,'FontSize',12)
set (myylabel,'FontSize',12)
title('Integrator and Observer')

figure
impulse(sys_cl, sys_integrator, sys_obv, sys_obv_steady)
legend( 'close loop', 'Integrator', 'observer', 'sys_obv_steady');



=======
L1 = 0.120;
L2 = 0.130;
I1 = 2.2e-4;
I2 = 2.5e-3;	
m1 = 0.22;
m2 = 0.085;
g = 9.81;

N = 1;
Ke = 1.66e-2;
Kt = 27.4e-3;
Rm = 12;

a = m1*L1^2 + m2*L2^2 + I1 ;
b =(m1*L1 + m2*L2) * g ;
a21= b/a ;
a24=(Kt*Ke*N^2)/ (a*Rm) ;
a41= - b/a ;
a44=((a+I2)/(a*I2))*((Kt*Ke*N^2)/ (Rm));  
b2 = -(Kt*N)/ (a*Rm); 
b4 = ((a+I2)/(a*I2))* ((Kt*N)/ (a*Rm)); 


A = [0    1   0   0;
     a21  0   0   a24;
     0    0   0   1;
     a41  0   0   a44];
 
B = [0;
     b2;
     0;
     b4];
 
C = [1 0 0 0];

D = [];


sys = ss(A,B,C,D,...
    'InputName','V','OutputName','y','StateName',{'x1','x2','x3','x4'});
% Controllability 
ControllabilityMatrix = ctrb(sys);
ControllabilityMatrixRank = rank(ControllabilityMatrix);

% Observability
ObservabilityMatrix = obsv(sys);
Rank = rank(ObservabilityMatrix);
Determinant = det(ObservabilityMatrix);

% Computation of controllr gains 
poleOL = pole(sys);
%pzmap(sys);
p = [ -8.73 -8.73 -2+2i -2-2i];
k = acker(A,B, p) 

%Define closed loop feedback system 
    % We must modify the C matrix to a 3x3 identity matrix for the feedback
    % function to work 
sys.c = eye(4);
closdLoopSys = feedback(sys,k);
poleCL = pole(closdLoopSys); %Matches the desired poles
step(closdLoopSys(1))


>>>>>>> 0a8996af9dabf98ba45f7b02f9f123a3b66a0d78
