function [arm_len,input_angle,base_lw,basecentre_xy,phi,th,psi,d] = robot_init()
    arm_len = [1,1,1,1,1,1];  %arm lengths
    input_angle = [-30,90,-20,0,-50,40];
    base_lw = [1,1];
    basecentre_xy = [0.001,0.001,0];
    phi = 0;
    th = 0;
    psi = 0;
    d = [ 0 ; 0 ];
end