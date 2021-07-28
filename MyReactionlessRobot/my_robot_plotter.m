function [ x_endeffector , y_endeffector ] = my_robot_plotter(len,input_angle,Base_dim,Base_lw)
l=len;  
ini_angle=input_angle;
len = Base_lw(1);
breadth = Base_lw(2);
%-------Base Centre x,y-----------%
xc = Base_dim(1);
yc = Base_dim(2);
%-------Base angle----------------%
th = 180/pi*Base_dim(4);
%-------Base at origin------------%
x=0;
y=0;
o = [x-len/2,y-breadth/2]; % base left corner
p = [o(1)    ,o(2)    ,0;
     o(1)+len,o(2)    ,0;
     o(1)+len,o(2)+len,0;
     o(1)    ,o(2)+len,0]; % Base four points
%-------Rotation Matrix for base-------------%
r = [cosd(th),-sind(th),0;
     sind(th), cosd(th),0;
           0 ,       0,1];% Base rotation matrix
%-------Rotating base using rotation matrix-------%
 a = r*p.';
%-------Translated rotated base to given Base centre-----%
 c = [a(1,:)+xc;a(2,:)+yc;a(3,:)]; %Base translated to new point
%-------Finding start of link--------------------%
 mid = r*[len/2,0,0].'; 
 start_link = [ mid(1)+xc,mid(2)+yc ];
%-------Finding link end positions---------------%
x=[(l(1)*cosd(ini_angle(1)+th)),(l(2).*cosd(ini_angle(1)+th+ini_angle(2))),(l(3).*cosd(ini_angle(1)+th+ini_angle(2)+ini_angle(3)))];
y=[(l(1)*sind(ini_angle(1)+th)),(l(2).*sind(ini_angle(1)+th+ini_angle(2))),(l(3).*sind(ini_angle(1)+th+ini_angle(2)+ini_angle(3)))];
a=[start_link(1)+0,start_link(1)+x(1),start_link(1)+x(1)+x(2),start_link(1)+x(1)+x(2)+x(3)];
b=[start_link(2)+0,start_link(2)+y(1),start_link(2)+y(1)+y(2),start_link(2)+y(1)+y(2)+y(3)];
% figure(100);
hold off;plot(0,0);
hold on;
%-------------plotting base--------------------%
line([c(1,:),c(1,1)],[c(2,:),c(2,1)],'LineWidth',2,'Color','k');
%------------Plotting links-------------------%
line(a(1:2),b(1:2),'LineWidth',3,'Color','r');
line(a(2:3),b(2:3),'LineWidth',3,'Color','g');
line(a(3:4),b(3:4),'LineWidth',3,'Color','b');
grid on 
axis([-3 4 -3 4]);
axis square
x_endeffector = a(4);
y_endeffector = b(4);
end