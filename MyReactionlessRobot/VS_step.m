function thetad = VS_step(t,theta,cam,p_desired,Link_Length,Base_Dim)
figure(200)
% hold on;
my_robot_plotter(Link_Length,180/pi.*theta(1:3),theta(4:9),Base_Dim);
lambda=0.01;
span=[4:6,1:3];
[Link_Length,Link_Mass,Basecentre_xy,Base_Dim,cube_p] = model_Input();
thr = [0;theta(1);theta(2);theta(3)];
qr = [theta(4);theta(5);theta(6);theta(7);theta(8);theta(9)];
phi0=qr(4);th0=qr(5);si0=qr(6);
Q1=[cos(phi0)*cos(si0)-sin(phi0)*sin(th0)*sin(si0)   -sin(phi0)*cos(th0)     cos(phi0)*sin(si0)+sin(phi0)*sin(th0)*cos(si0)
    sin(phi0)*cos(si0)+cos(phi0)*sin(th0)*sin(si0)    cos(phi0)*cos(th0)     sin(phi0)*sin(si0)-cos(phi0)*sin(th0)*cos(si0)
                                -cos(th0)*sin(si0)              sin(th0)                                  cos(th0)*cos(si0)];

[n,nq,a,b,alp,bt,al,alt,dx,dy,dz,m,g,Icxx,Icyy,Iczz,Icxy,Icyz,Iczx,ane,ee] = robot_description();

frames = 20;
for iter = 1:frames
   centre = mean(cube_p);
   rotation = AxelRot(cube_p',20,[ 0 1 0 ], centre); %Arg1 - 3xN, Arg2 - deg, Arg3-dir, Arg4-1x3 centre pt
   cube_p = rotation'; %new cube pts
   %Points in Camera frame
   Pc = Point_in_CameraFrame(theta,cube_p'); %[x;y;z]
   %Points in image coordiates
   p = Point_to_ImageCoord(cam,Pc);
   xy1 = p;
   xy1_arr(:,:,iter)=xy1;
end
%Let's generate the ellipse!
[mat_ellipse,error] = ellipse_generator(xy1, xy1_arr, p_desired);
z_ellipse = [mat_ellipse(:,1),mat_ellipse(:,2)]';
a_ellipse = mat_ellipse(:,3);
b_ellipse = mat_ellipse(:,4);
alpha_ellipse = mat_ellipse(:,5);
ellipse_error = mat_ellipse(:,6);
err = norm(ellipse_error);
% error_history = [error_history,err];
Z = 1; %We consider the camera to be in the XY plane, so Z is taken to be 1
Lsd=[];
for i0=1:size(error,2)
    Lsd=[Lsd;getinteraction_ellipse(cam,Z,1,Pc(3,:),z_ellipse(:,i0), a_ellipse(i0), b_ellipse(i0), alpha_ellipse(i0),xy1(1,i0),xy1(2,i0))];
end
%velocities in the camera frame
vc = control_algorithm(Lsd,error);
R = [  cos( theta(1) + theta(2) + theta(3) ) , - sin( theta(1) + theta(2) + theta(3) ) , 0 ;
       sin( theta(1) + theta(2) + theta(3) ) ,   cos( theta(1) + theta(2) + theta(3) ) , 0 ;
                                                 0 ,                                             0 , 1 ];
R = [R zeros(3,3);zeros(3,3) R];

%velocities of the camera wrt satellite frame
vc = R*vc;                                             
% vc_history = [vc_history;vc'];
[Ib Im Ibm Jbe Jme GJM Jg2] = Jacobian_vom(qr,thr,n,alp,a,b,bt,dx,dy,dz,m,Icxx,Icyy,Iczz,Icxy,Icyz,Iczx, ee, ane);
invT = inv(R);
Lm = Lsd * invT * GJM(span,:);
E1 = error(:);
work1 = -pinv(Lm)*E1;
HpposHp = pinv(Lm)*(Lm);
E2=zeros(n-1,1);
work2=zeros(n-1,1);
E=work1+work2;
dthr3=-lambda*work1;
dthr=dthr3;
dt0r=pinv(Ib)*(-Ibm'*dthr);
ibmdthr=Ibm'*dthr;
hbmtildatd=zeros();
dq=dt0r;
dth=dthr;
Ibm1=Ibm.';
Ict=Ibm1(4:6,:)-Ib(4:6,1:3)*Ibm1(1:3,:)/Ib(1,1);
options=optimset;
[E2,costminsearch]=fminsearch(@minimizeE2,E2,options,lambda,work1,HpposHp,Ib,Ibm,Ict);
e_thetad =  ( 1 *pinv(Lm) * E1) +(eye(size(HpposHp))-HpposHp)*E2;
te = GJM*e_thetad;
tb = inv(Jbe)*(te-(Jme * e_thetad));
P=[    -sin(si0)*cos(th0)   cos(si0)  0
        sin(th0)            0         1
        cos(si0)*cos(th0)   sin(si0)  0    ];
b_thetad = inv(P)*[tb(4);tb(5);tb(6)];
b_dim = [tb(1:3);b_thetad];
e_thetad
thetad = [e_thetad;b_dim];
% GJM
% GJM1 = GJM(span,:);
% Rbc = R';
% Lm = Lsd*Rbc*GJM1;
% thetad =  - 1 *pinv(Lm)*E1;
% work1 = thetad;
% HpposHp = pinv(Lm)*(Lm);
% E2 = [0;0;0];
% Ibm1=Ibm.';
% Ict=Ibm1(4:6,:)-Ib(4:6,1:3)*Ibm1(1:3,:)/Ib(1,1);
% options=optimset;
% [E2,costminsearch]=fminsearch(@minimizeE2,E2,options,lambda,work1,HpposHp,Ib,Ibm,Ict);
% e_thetad =  ( -1 *pinv(Lm) * E1) +(eye(size(HpposHp))-HpposHp)*E2;
% te = GJM*e_thetad;
% tb = inv(Jbe)*(te-(Jme * e_thetad));
% P=[    -sin(si0)*cos(th0)   cos(si0)  0
%         sin(th0)            0         1
%         cos(si0)*cos(th0)   sin(si0)  0    ];
% b_thetad = inv(P)*[tb(4);tb(5);tb(6)];
% b_dim = [tb(1:3);b_thetad];
% err
% thetad = [e_thetad;b_dim];
end