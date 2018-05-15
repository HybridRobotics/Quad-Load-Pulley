function[dx, xLd, Rd, qd, ld, f, M, tau] = odefun_control(t,x,data,obsta)
%% Constants
mL = data.params.mL ;
g = data.params.g ;
mQ = data.params.mQ ;
J = data.params.J ;
e1 = data.params.e1 ;
e2 = data.params.e2 ;
e3 = data.params.e3 ;
a = data.params.a;
Jp = data.params.Jp;

%% Desired States
%---------------%
xLd = zeros(3,1);
vLd = zeros(3,1);
aLd = zeros(3,1);
% xLd = [1-cos(t);sin(t);0];
% vLd = [sin(t);cos(t);0];
% aLd = [cos(t);-sin(t);0];
xLd = [ 3*sin(t); 3-1*cos(t);0];
vLd = [ 3*cos(t); 1*sin(t);0];
aLd = [-3*sin(t); 1*cos(t);0];
% th = -165*pi/180 ;
qd = [0;0;-1];
% qd = [ 0; sin(th); cos(th)];
dqd = zeros(3,1);
d2qd = zeros(3,1);
Rd = eye(3);
Omegad = zeros(3,1);
dOmegad = zeros(3,1);
% ld = 1;
% dld = 0;
% ddld = 0;
dx = [];

% Case 1: Testing
%[xLd,vLd,aLd,ld,dld,ddld,qd,dqd,d2qd,~,Omegad,dOmegad] = get_nom_traj(data.params, get_load_traj(t));

% Case 2: Window Passing
[xLd,vLd,aLd,ld,dld,ddld,qd,dqd,d2qd,~,Omegad,dOmegad] = get_nom_traj(data.params,get_load_traj2(t));

%% Extracting States
xL = x(1:3);
vL = x(4:6);
q = x(7:9); 
omega = x(10:12); 
dq = vec_cross(omega, q);
R = reshape(x(13:21), 3,3);
Omega = x(22:24);
l = x(25);
dl = x(26);
b3 = R(:,3);
b1 = R(:,1);

%% Error Function of Cable Length
el = l - ld; del = dl - dld;

%% Error Function of Load Position
eL = xL - xLd; deL = vL - vLd;

%% Parameters of two controllers
kl = 10; kdl = 1;
kx = diag([1 1 2]); kv = diag([2 2 1.5]);

%% Parameters of System
D = [diag(repmat(mQ+mL,3,1)),-mQ*q;-a*mL*q',-Jp/a];

%% Controller of Load Position
temp = D*[vLd+g*e3-kx*eL-kv*deL;ddld-kl*el-kdl*del] + [mQ*l*vec_dot(dq,dq)*q;0];
A = temp(1:3); tau = temp(4);
qd = -A/norm(A);

%% Load Attitude Controller
epsilon_q = 0.5;
err_q = hat_map(q)^2*qd;
kp = 1.5/epsilon_q^2 ; kom = 0.90/epsilon_q;
err_om = dq - vec_cross(vec_cross(qd, dqd), q);
F_pd = -kp*err_q-kom*err_om;
F_ff = mQ*l*(vec_dot(q,vec_cross(qd,dqd))*vec_cross(q,dq) + ...
    vec_cross(vec_cross(qd,d2qd),q)- 2*(dl/l)*vec_cross(q,omega));
F_n = vec_dot(A,q)*q;
F = -F_pd - F_ff + F_n;
b3c = F / norm(F);
f = vec_dot(F, R(:,3));

%% Return to the normalized dynamical equations and Get values of l_dot dl_dot xL_dot vL_dot
H = [q*(vec_dot(q,f*b3)-mQ*l*vec_dot(dq,dq));tau];
temp2 = inv(D)*H-[g*e3;0];
l_dot = dl;
dl_dot = temp2(4);
xL_dot = vL;
vL_dot = temp2(1:3);

%% Quadrotor Attitude Controller
b1d = e1;
b1c = -vec_cross(b3c,vec_cross(b3c,b1d));
b1c = b1c/norm(vec_cross(b3c,b1d));
Rc = [b1c vec_cross(b3c,b1c) b3c];
Rd = Rc;
if(norm(Rd'*Rd-eye(3)) > 1e-2)
    disp('Error in R') ; keyboard ;
end
kR = 4; kOm = 4;
epsilon = 0.1;
err_R = 1/2 * vee_map(Rd'*R - R'*Rd);
err_Om = Omega - R'*Rd*Omegad;
M = -kR/epsilon^2*err_R - kOm/epsilon*err_Om + vec_cross(Omega, J*Omega)...
    -J*(hat_map(Omega)*R'*Rd*Omegad - R'*Rd*dOmegad)+tau*b1;
%% Saturation
M(M>2)=2;
M(M<-2)=-2;
f(f>10)=10;
f(f<0)=0;
tau (tau>0.1)=0.1;

%% Quadrotor Attitude Dynamics
R_dot = R*hat_map(Omega);
q_dot = dq;
omega_dot = -(1/(mQ*l))*(vec_cross(q,f*b3) + 2*mQ*dl*omega);
Omega_dot = J\( -vec_cross(Omega, J*Omega)+M-tau*b1);
dx = [xL_dot; vL_dot; q_dot; omega_dot; reshape(R_dot,[9,1]); Omega_dot;...
      l_dot; dl_dot];
disp(eL)

if nargout <= 1
   fprintf('Simulation time %0.4f seconds \n',t);
end
end