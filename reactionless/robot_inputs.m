function [n,nq,a,b,alp,bt,al,alt,dx,dy,dz,m,g,Icxx,Icyy,Iczz,Icxy,Icyz,Iczx] = robot_inputs()
n = 8; %no. of links  = 3 links x 2 arms + base_link
nq=0;%1 for spatial and 0 for planar
%ENTER DH PARAMETER HERE   
%  dh=[a d alp th];
a = [0.5,1,1,1,-0.5,-1,-1,-1]'; %Link lengths between zj and zj-1 axes along xj axis    #DOUBT 
b = [0; 0; 0; 0; 0; 0; 0; 0]; %prismatic parameter (Link offset)
alp = [0; 0; 0; 0; 0; 0; 0; 0];
%Parent array bt and corrosponding vectors
bt=[0 1 2 3 4 5 6 7];
%Link Lengths
al=[1; 1; 1; 1; 1; 1; 1; 1]; %AL
alt=[0.5; 1; 1; 1; -0.5; -1; -1; -1]; %Link end positions
%ENTER VECTOR dm 
dx=[  0    al(2)/2  al(3)/2  al(4)/2   0   -al(5)/2  -al(6)/2  -al(7)/2 ];
dy=[  0       0       0       0    0       0       0       0  ];
dz=[  0       0       0       0    0       0       0       0  ];
m=[500; 10; 10; 10; 500; 10; 10; 10 ];
g=[0 ; 0; 0];
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
Icxx(1)=83.61;  Icyy(1)=83.61; Iczz(1)=83.61;
Icxx(5)=83.61;  Icyy(5)=83.61; Iczz(5)=83.61;
Icxx(2)=0.01;   Icyy(2)=1.05;  Iczz(2)=1.05;
Icxx(3)=0.01;   Icyy(3)=1.05;  Iczz(3)=1.05;
Icxx(4)=0.01;   Icyy(4)=1.05;  Iczz(4)=1.05;
Icxx(6)=0.01;   Icyy(6)=1.05;  Iczz(6)=1.05;
Icxx(7)=0.01;   Icyy(7)=1.05;  Iczz(7)=1.05;
Icxx(8)=0.01;   Icyy(8)=1.05;  Iczz(8)=1.05;

end