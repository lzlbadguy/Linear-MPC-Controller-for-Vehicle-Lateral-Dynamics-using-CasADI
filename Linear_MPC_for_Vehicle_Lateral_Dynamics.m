clc;
clear;
%% Constant
lf=1.2;
lr=1.6;
vx=30/3.6;
%% Casadi define
import casadi.*
Ts_p = 0.05;
Np = 10;
Nc = 5;
n_states = 4;
n_controls = 1;

P = SX.sym('P',n_states + 2*Np);
U = SX.sym('U',n_controls,Nc);
X = SX.sym('X',n_states,(Np+1));

X(:,1) = P(1:n_states);
for k = 1:Np
    if k<=Nc
        X(:,k+1) = vehicle_lateral_dynamics(X(:,k),U(k),Ts_p,vx);
    else
        X(:,k+1) = vehicle_lateral_dynamics(X(:,k),U(Nc),Ts_p,vx);
    end
end

obj = 0; % Objective function
g = [];  % constraints vector

Q1 = 1000;
Q2 = 100;
R=1;

% compute objective
for k=1:Np
    obj = obj+(X(4,k+1)-P(n_states+k))'*Q1*(X(4,k+1)-P(n_states+k))/1.75^2;
    obj = obj+(X(2,k+1)-P(n_states+Np+k))'*Q2*(X(2,k+1)-P(n_states+Np+k))/(pi/18)^2;
    if k<Nc
        obj = obj+(U(1,k))'*R*(U(1,k))/(pi/18)^2;
    end
end

% compute constraints
for k = 1:Np+1   
    g = [g ; X(2,k)];   %state psi
    g = [g ; X(4,k)];   %state Y
    if k>1 && k<=Nc
        g = [g ; (U(k)-U(k-1))]; 
    end
end

% make the decision variables one column vector
OPT_variables = reshape(U,Nc,1);
% nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);
qp = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);



% solver = nlpsol('solver', 'ipopt', nlp_prob,opts);
% opts = struct;
% opts.qpoases.printLevel ='PL_NONE';
solver = qpsol('solver', 'qpoases', qp);

args = struct;
ubg=[];
lbg=[];

for k=1:Np+1
    lbg=[lbg;-pi/6];
    lbg=[lbg;-4];
    ubg=[ubg;pi/6];
    ubg=[ubg;4];
        if k>1 && k<=Nc
            lbg=[lbg;-pi/36*Ts_p];
            ubg=[ubg;pi/36*Ts_p];
        end
end
args.ubg=ubg;
args.lbg=lbg;

ubx=[];
lbx=[];

for k=1:Nc
   ubx=[ubx;pi/18]; 
   lbx=[lbx;-pi/18]; 
end

args.ubx=ubx;
args.lbx=lbx;

%% THE SIMULATION LOOP SHOULD START FROM HERE
t = 0;
x=[0;0;0;0];
xp = 0;
% xs = 1.75; % Reference posture.
u=0;

th=[t];
xh=[x];
xph=[xp];
uh=[u];
d_x1=25;
d_x2=21.95;
d_y1=4.05;
d_y2=5.7;
z1 = 2.4/25*(xp-27.19)-1.2;
z2 = 2.4/21.95*(xp-56.46)-1.2;
Y_r=d_y1/2*(1+tanh(z1))-d_y2/2*(1+tanh(z2));
psi_r=atan(d_y1*(1/cosh(z1))^2*(1.2/d_x1)-d_y2*(1/cosh(z2))^2*(1.2/d_x2));

Y_rh=[Y_r];
psi_rh=[psi_r];
simtime=15;
Ts=0.05;
u0 = zeros(Nc,1);
for i=1:(simtime/Ts)
    tic;
    % z1 = 2.4/25*(xp-27.19)-1.2;
    % z2 = 2.4/21.95*(xp-56.46)-1.2;
    Y_r_pre=zeros(Np,1);
    psi_r_pre=zeros(Np,1);
    for j=1:Np
        xp_pre(j)=xp+vx*(j-1)*Ts;
        z1_pre(j) = 2.4/25*(xp_pre(j)-27.19)-1.2;
        z2_pre(j) = 2.4/21.95*(xp_pre(j)-56.46)-1.2;
        Y_r_pre(j)=d_y1/2*(1+tanh(z1_pre(j)))-d_y2/2*(1+tanh(z2_pre(j)));
        psi_r_pre(j)=atan(d_y1*(1/cosh(z1_pre(j)))^2*(1.2/d_x1)-d_y2*(1/cosh(z2_pre(j)))^2*(1.2/d_x2));
    end
    xs=[Y_r_pre;psi_r_pre];
    args.p   = [x;xs];
    args.x0 = reshape(u0',Nc,1);
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);   
    uout = reshape(full(sol.x)',1,Nc)';
    u=uout(1);  
    xup=vehicle_lateral_dynamics(x,u,Ts,vx);
    x = xup;
    xp=xp+vx*Ts;
    t=t+Ts;
           
    u0 = [uout(2:Nc);uout(Nc)];
    th=[th,t];
    xh=[xh,x];
    uh=[uh,u];
    Y_rh=[Y_rh,Y_r_pre(1)];
    psi_rh=[psi_rh,psi_r_pre(1)];
    xph=[xph,xp];
    sample_time(i)=toc;
end
Simulation_run_time=sum(sample_time)
figure
plot(xph,xh(4,:),'LineWidth',2);
hold on;
plot(xph,Y_rh,'--','LineWidth',2);
xlabel('$X$','Interpreter','latex','FontSize',15);
ylabel('$Y$','Interpreter','latex','FontSize',15);
legend('$Y$','$Y^{r}$','Interpreter','latex','FontSize',15);

figure
plot(xph,xh(2,:),'LineWidth',2);
hold on;
plot(xph,psi_rh,'--','LineWidth',2);
xlabel('$X$','Interpreter','latex','FontSize',15);
ylabel('$\psi$','Interpreter','latex','FontSize',15);
legend('$\psi$','$\psi^{r}$','Interpreter','latex','FontSize',15);

