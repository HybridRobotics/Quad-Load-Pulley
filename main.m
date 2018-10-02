%% Quadrotor Attitude Pendulum SO(3)x S(2)x R - Dynamics Simulation and Geometry Control
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

%% Get initial condition from the nominal trajectory
xL = [0.5;-0.5;2.5-0.5];
vL = zeros(3,1);
th = 135*pi/180;
q = [ sin(th);0;cos(th)];
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
% we use polynomial function to generate aimed cable length generation
T = 10;
WindowNum = [0,0.25*T,0.5*T,0.75*T,T];
LengthNum = [0.25,0.2,0.1,0.2,0.25];
% Line Widths
lw1 = 5 ; lw2 = 2 ; lws = 3 ;

% colors
c1 = 'r' ; c2 = 'k' ; cs = 'g' ;

% Gap for subplot1
subplot1_gap = [0 0];
subplot1a_gap = [0 0.08];

% Paper position size
paperpos_sz_1fig = [0.25 2.5 8 2];%2.4] ;
paperpos_sz_2fig = [0.25 2.5 8 3];%4] ;
paperpos_sz_4fig = [0.25 2.5 8 8.5];

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
subplot1(1,1, 'Gap', subplot1_gap);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperPosition', paperpos_sz_1fig);
plot(t(ind), M(ind,1),'r'); hold on;
plot(t(ind), M(ind,2),'b');
plot(t(ind), M(ind,3),'y');
legend({'$$M_1$$','$$M_2$$','$$M_3$$'},'interpreter','latex');
title('Input Moment');
grid on ; 
xlabel('Time (s)');

% Input Thrust
figure;
subplot1(1,1, 'Gap', subplot1_gap);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperPosition', paperpos_sz_1fig);
plot(t(ind), f(ind,1));
legend({'$$f$$'},'interpreter','latex');
title('Input Thrust');
grid on;
xlabel('Time (s)');

% Input Torque
figure;
subplot1(1,1, 'Gap', subplot1_gap);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperPosition', paperpos_sz_1fig);
plot(t(ind), tau(ind,1)) ;
legend({'$$\tau$$'},'interpreter','latex');
title('Input Torque');
grid on;
xlabel('Time (s)');

% Configuration Errors
figure;
subplot1(1,1, 'Gap', subplot1_gap);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperPosition', paperpos_sz_1fig);
plot(t(ind), psi_R(ind)) ;
grid on;
xlabel('Time (s)');
title('$$\psi_R$$','interpreter','latex');

figure;
subplot1(1,1, 'Gap', subplot1_gap);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperPosition', paperpos_sz_1fig);
plot(t(ind), psi_q(ind)) ;
grid on;
xlabel('Time (s)');
title('$$\psi_q$$','interpreter','latex');

figure;
subplot1(1,1, 'Gap', subplot1_gap);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperPosition', paperpos_sz_1fig);
plot(t(ind),psi_exL(ind));
grid on;
xlabel('Time (s)');
title('$$||e_{x_L}||$$','interpreter','latex');

figure;
subplot1(1,1, 'Gap', subplot1_gap);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperPosition', paperpos_sz_1fig);
plot(t(ind),el(ind));
grid on;
xlabel('Time (s)');
title('cable length error');

figure;
subplot1(1,1, 'Gap', subplot1_gap);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperPosition', paperpos_sz_1fig);
plot(t(ind),psi_exL1(ind),'r-'); hold on;
plot(t(ind),psi_exL2(ind),'y.-');
plot(t(ind),psi_exL3(ind),'b--');
grid on;
legend('x','y','z')
xlabel('Time (s)');
title('Position Error');

% Load Position
figure;
subplot1(1,1, 'Gap', subplot1_gap);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperPosition', paperpos_sz_1fig);
plot(t, x(:, 1:3)) ;
grid on; title('xL');
legend('x','y','z');
xlabel('Time (s)') ;

% Quadrotor Attitude
figure;
subplot1(1,1, 'Gap', subplot1_gap);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperPosition', paperpos_sz_1fig);
plot(t(ind), phi_d(ind)*180/pi, 'b:'); hold on;
plot(t(ind), theta_d(ind)*180/pi, 'g:');
plot(t(ind), psi_d(ind)*180/pi, 'r:'); 
plot(t(ind), phi(ind)*180/pi, 'b'); 
plot(t(ind), theta(ind)*180/pi, 'g'); 
plot(t(ind), psi(ind)*180/pi, 'r') ;
grid on; 
xlabel('Time (s)');
ylabel('deg');
title('Quad Atttitude') ;
legend({'$$\phi_d$$','$$\theta_d$$','$$\psi_d$$','$$\phi$$','$$\theta$$','$$\psi$$'},'interpreter','latex');

% Load Attitude
figure;
plot(t(ind), qd(ind,1), 'b:',t(ind), qd(ind,2), 'g:',t(ind), qd(ind,3), 'r:') ;
hold on;
plot(t(ind), x(ind, 7),'b',t(ind), x(ind, 8),'g',t(ind), x(ind, 9),'r') ;
grid on;
xlabel('Time (s)');
title('Load Attitude');
legend({'$$q_{d1}$$','$$q_{d2}$$','$$q_{d3}$$','$$q_1$$','$$q_2$$','$$q_3$$'},'interpreter','latex');

%% Animation
animate_3dquad_load(t, x, t(ind), xLd(ind,:));