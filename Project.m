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


