%% Quadrotor Attitude Pendulum SO(3)x S(2)x R - Dynamics Simulation and Geometry Control
close all;
%% Parameters
data.params.mQ = 0.5;
data.params.mL = 0.087;
data.params.J = [2.32e-3,0,0;0,2.32e-3,0;0,0,4e-3];
data.params.g = 9.81;
data.params.e1 = [1;0;0];
data.params.e2 = [0;1;0];
data.params.e3 = [0;0;1];
data.params.a = 0.03;
data.params.Jp = 3e-4;

data.Model_Flag = "Window Passing";

%% Get initial condition from the nominal trajectory
xL = [0.5;-0.5;2.5-0.5];
xL = [0;0;2.5];
vL = zeros(3,1);
th = 135*pi/180;
q = [-sin(th);0;cos(th)];
omega = [0;0;0];
R = eye(3,3);
Omega = [0;0;0];
l = 1;
dl = 0;

%% In-Sanity checks on initial condition
if(abs(norm(q)-1) > 1e-2)
    disp('Error in q') ; keyboard ;
end

x_0 = [xL; vL; q; omega; reshape(R, 9,1); Omega; l; dl];

%% Window Passing Case (Cable Length Generation)
% if data.Model_Flag == "Window Passing"
% we use polynomial function to generate aimed cable length generation
T = 10;
WindowNum = [0,0.25*T,0.5*T,0.75*T,T];
LengthNum = [0.25,0.2,0.1,0.2,0.25];

Res = polyfit(WindowNum,LengthNum,4);
TimeSpan = linspace(0,T,1000);
plot(TimeSpan,polyval(Res,TimeSpan),'-',WindowNum(2:4),LengthNum(2:4),'x');
legend('Interpolated flat output','Three Windows')
syms l_interp  dl_interp d2l_interp d3l_interp d4l_interp t
data.obsta.l_interp = Res(1)*t^4+ Res(2)*t^3+ Res(3)*t^2+ Res(4)*t+ Res(5);
data.obsta.dl_interp = diff(data.obsta.l_interp,1);
data.obsta.d2l_interp = diff(data.obsta.l_interp,2);
data.obsta.d3l_interp = diff(data.obsta.l_interp,3);
data.obsta.d4l_interp = diff(data.obsta.l_interp,4);
% end

%% Solving Dynamical Equations
odeopts = [];
[t, x] = ode45(@odefun_control, [0 10], x_0, odeopts, data);

%% Compute various quantities

disp('Computing...') ;
ind = round(linspace(1, length(t), round(0.1*length(t)))) ;
for j=ind
    [~, xLd_, Rd, qd_, ld_, f_, M_, tau_] = odefun_control(t(j), x(j,:)', data) ;
    [phi_d(j),theta_d(j),psi_d(j)] = RotToRPY_ZXY(Rd) ;
    [phi(j),theta(j),psi(j)] = RotToRPY_ZXY(reshape(x(j,13:21),3,3));
    xLd(j,:)=xLd_';
    M(j,:) = M_' ;
    f(j,:) = f_' ;
    qd(j,:) = qd_';
    el(j) = norm(x(j,25)'-ld_');
    psi_q(j) = 1 - qd_'*x(j, 7:9)' ;
    psi_R(j) = 0.5*trace(eye(3,3) - Rd'*reshape(x(j,13:21),3,3)) ;
    psi_exL(j) = norm(x(j,1:3)-xLd_');
    psi_exL1(j) = x(j,1)-xLd_(1);
    psi_exL2(j) = x(j,2)-xLd_(2);
    psi_exL3(j) = x(j,3)-xLd_(3);
    
    tau(j,:) = tau_;
end

disp('Computing...') ;
ind = round(linspace(1, length(t), round(0.1*length(t)))) ;

%% Plotting various states/outputs/inputs

% Input Moment
figure;
plot(t(ind), M(ind,1),t(ind), M(ind,2),t(ind), M(ind,3)) ;
legend('M(1)','M(2)','M(3)');title('Quad-Moment');
grid on ; xlabel('Time (s)') ;

% Input Thrust
figure;
plot(t(ind), f(ind,1)) ;
legend('f'); title('Quad-Thrust');
grid on ; xlabel('Time (s)')

% Input Torque
figure;
plot(t(ind), tau(ind,1)) ;
legend('\tau'); title('Pulley-Torque');
grid on ; xlabel('Time (s)')

% Configuration Errors
figure ; plot(t(ind), psi_R(ind)) ;
grid on ; xlabel('Time (s)') ; title('psi-R')
figure ; plot(t(ind), psi_q(ind)) ;
grid on ; xlabel('Time (s)') ; title('psi-q')
figure; plot(t(ind),psi_exL(ind));
grid on; xlabel('Time (s)'); title('position error');
figure; plot(t(ind),el(ind));
grid on; xlabel('Time (s)'); title('cable length error');
figure; plot(t(ind),psi_exL1(ind),t(ind),psi_exL2(ind),t(ind),psi_exL3(ind))
grid on; xlabel('Time (s)'); title('Position Error');
% Load Position
figure ; plot(t, x(:, 1:3)) ;
grid ; title('xL') ; legend('x','y','z') ; xlabel('Time (s)') ;

% Quadrotor Attitude
figure ; plot(t(ind), phi_d(ind)*180/pi, 'b:', t(ind), theta_d(ind)*180/pi, 'g:', t(ind), psi_d(ind)*180/pi, 'r:') ;
hold on ; plot(t(ind), phi(ind)*180/pi, 'b', t(ind), theta(ind)*180/pi, 'g', t(ind), psi(ind)*180/pi, 'r') ;
grid ; xlabel('Time (s)') ; ylabel('deg') ; title('Quad Atttitude') ;
legend('phi_d','theta_d','psi_d','phi','theta','psi');

% Load Attitude
figure ; plot(t(ind), qd(ind,1), 'b:',t(ind), qd(ind,2), 'g:',t(ind), qd(ind,3), 'r:') ;
hold on ; plot(t(ind), x(ind, 7),'b',t(ind), x(ind, 8),'g',t(ind), x(ind, 9),'r') ;
xlabel('Time (s)');title('Load Attitude') ; grid ;
legend('q_d1','q_d2','q_d3','q_1','q_2','q_3');

%% Animation
animate_3dquad_load(t, x, t(ind), xLd(ind,:));