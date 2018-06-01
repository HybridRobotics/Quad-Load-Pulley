 function load_traj = get_load_traj2(t)
%% Test: Circular Displacement  
a_x = 3;
a_y = 3;
a_z = 2;
f = 0.1;
%% Desired Load Position Generation
load_traj.xL = [a_x*(1 - cos(2*pi*f*t));
    a_y * sin(2*pi*f*t);
    a_z * sin(2*pi*f*t)+2.5];
load_traj.dxL = 2*pi*[f*a_x*sin(2*pi*f*t);
    f*a_y * cos(2*pi*f*t);
    f*a_z * cos(2*pi*f*t)];
load_traj.d2xL = (2*pi)^2*[f^2*a_x*cos(2*pi*f*t);
    f^2*a_y * -sin(2*pi*f*t);
    f^2*a_z * -sin(2*pi*f*t)];
load_traj.d3xL = (2*pi)^3*[f^3*a_x*-sin(2*pi*f*t);
    f^3*a_y * -cos(2*pi*f*t);
    f^3*a_z * -cos(2*pi*f*t)];
load_traj.d4xL = (2*pi)^4*[f^4*a_x*-cos(2*pi*f*t);
    f^4*a_y * sin(2*pi*f*t);
    f^4*a_z * sin(2*pi*f*t)];
load_traj.d5xL = (2*pi)^5*[f^5*a_x*sin(2*pi*f*t);
    f^5*a_y * cos(2*pi*f*t);
    f^5*a_z * cos(2*pi*f*t)];
load_traj.d6xL = (2*pi)^6*[f^6*a_x*cos(2*pi*f*t);
    f^6*a_y * -sin(2*pi*f*t);
    f^6*a_z * -sin(2*pi*f*t)];

T = 1/f;
load_traj.l = - t^4/1875 + (4*t^3)/375 - (91*t^2)/1500 + (11*t)/150 + 1/4;
load_traj.dl = - (4*t^3)/1875 + (4*t^2)/125 - (91*t)/750 + 11/150;
load_traj.d2l = (8*t)/125 - (4*t^2)/625 - 91/750;
load_traj.d3l = 8/125 - (8*t)/625;
load_traj.d4l = -8/625;
    
end