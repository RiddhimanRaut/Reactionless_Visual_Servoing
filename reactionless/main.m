clear;
clc;
[arm_len,input_angle,base_lw,basecentre_xy,phi,th,psi,d] = robot_init();
[cam_extr_stack,theta] = my_robot(arm_len,input_angle,base_lw,basecentre_xy,phi,th,psi);
[n,nq,a,b,alp,bt,al,alt,dx,dy,dz,m,g,Icxx,Icyy,Iczz,Icxy,Icyz,Iczx] = robot_inputs();
thr = [0;theta(1);theta(2);theta(3);180;theta(1);theta(2);theta(3)];
qr = [theta(7);theta(8);theta(9);theta(10);theta(11);theta(12)];
phi0=qr(4);th0=qr(5);si0=qr(6);
Q1=[cos(phi0)*cos(si0)-sin(phi0)*sin(th0)*sin(si0)   -sin(phi0)*cos(th0)     cos(phi0)*sin(si0)+sin(phi0)*sin(th0)*cos(si0)
        sin(phi0)*cos(si0)+cos(phi0)*sin(th0)*sin(si0)    cos(phi0)*cos(th0)     sin(phi0)*sin(si0)-cos(phi0)*sin(th0)*cos(si0)
                                    -cos(th0)*sin(si0)              sin(th0)                                  cos(th0)*cos(si0)];
ee=[n];
ane=[1 
     0 
     0 ];


Pt = [-2;1;100];
cam_extr_right = cam_extr_stack(1:3,:);
cam_extr_left = cam_extr_stack(4:6,:);
imgxy_right = point_to_image_coord(cam_extr_right,Pt);
imgxy_left = point_to_image_coord(cam_extr_left,Pt);
figure(2)
plot([imgxy_right(1),imgxy_left(1),0],[imgxy_right(2),imgxy_left(2),0],'o');
axis([-512 512 -512 512])

[Ib Im Ibm Jbe Jme GJM Jg2] = Jacobian_vom(qr,thr,n,alp,a,b,bt,dx,dy,dz,m,Icxx,Icyy,Iczz,Icxy,Icyz,Iczx, ee, ane);
GJM