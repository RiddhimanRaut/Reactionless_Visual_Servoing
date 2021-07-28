clear;
clc;
clf;
% lambda = 0.4;

%Initial positiion of the robot
[ Link_Length,Link_Mass,Basecentre_xy,Base_Dim,cube_p, p_desired,cam] = model_Input();
angle = [0*pi/180,0*pi/180 , 0*pi/180];
qr = [Basecentre_xy(1),Basecentre_xy(2),0,0,0,0];
theta = [angle,qr];

thr = [0;theta(1);theta(2);theta(3)]; %4 angles for the 4 links --> initialisation
qr = [theta(4);theta(5);theta(6);theta(7);theta(8);theta(9)];  %Base pose --> [x, y, z, roll, pitch, yaw]
% t = 1;
error_history = [];
while(1)
    [ t1, theta_array]= ode45( @(t,theta) VS_step(t,theta,cam,p_desired,Link_Length,Base_Dim), [0 1], theta );
end
% for i=1:length(t1)
%     figure(100)
%     my_robot_plotter(Link_Length,180/pi.*theta_array(i,1:3),theta_array(i,4:9),Base_Dim)
%     pause(0.1)
% end


