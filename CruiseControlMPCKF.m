clear all;
clc;

%% System modeling
dt = 0.1;
T  = 100;

d = 50;
m = 1000;

a = 1-d/m*dt;
b = dt/m;

%% Initialization
x0     = 0;
xArray = [];
x      = [x0 0 0 0 0 0 0 0]'; 
y      = 0;
yArray = [];

%% Kalman
P = 100;
C = 1;
Q = 0;
R = 1;

%% Constraint
xlow  = -10;
ulow  = 0;
xhigh = 100;
uhigh = 10000;

lb = [xlow*ones(4,1); ulow*ones(4,1)];
ub = [xhigh*ones(4,1); uhigh*ones(4,1)];

%% Weights
q = 10000;
r = 0.001;

%% Input reference
xref = 20;

%% Simulation

G = [q 0 0 0 0 0 0 0;
     0 q 0 0 0 0 0 0;
     0 0 q 0 0 0 0 0;
     0 0 0 q 0 0 0 0;
     0 0 0 0 r 0 0 0;
     0 0 0 0 0 r 0 0;
     0 0 0 0 0 0 r 0;
     0 0 0 0 0 0 0 r];
Aeq = [1 0 0 0 -b 0 0 0;
       -a 1 0 0 0 -b 0 0;
       0 -a 1 0 0 0 -b 0;
       0 0 -a 1 0 0 0 -b];

f   = [-q*xref*ones(4,1);zeros(4,1)];

%%
for i = 1:T
    xArray = [xArray x];
    yArray = [yArray y];
    
    beq = [a*x0 0 0 0]';
    x   = quadprog(G,f,[],[],Aeq,beq,lb,ub);
    
    y = a*x0+b*x(5)+ sqrt(R)*randn; % output
    
    % predict
    x0  = a*x0+b*x(5);
    P   = a*P*a' + Q;
    % update
    K   = P*C'*inv(C*P*C'+R);
    x0  = x0 + K*(y-C*x0);
    P   = (eye(1)-K*C)*P;
end

%% Plotting
figure(1)
subplot(2,1,1)
plot(dt:dt:T*dt,xArray(1,:),'o:')
ylim([0 25])
grid on;
subplot(2,1,2)
stairs(dt:dt:T*dt,xArray(5,:), 'o-')
grid on;
figure(2)
subplot(2,1,1)
plot(dt:dt:T*dt,xArray(1,:),'o:')
ylim([0 25])
grid on;
subplot(2,1,2)
stairs(dt:dt:T*dt,yArray, 'o-')
grid on;