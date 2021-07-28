function [ Link_Length,Link_Mass,Basecentre_xy,Base_Dim,cube_p, p_desired,cam] = model_Input()
    cam.K=[800 0 320; 0 800 240; 0 0 1];
    Link_Length = [1,1,1];
    Link_Mass = [1,1,1];
    Basecentre_xy = [ 0.0001 , 0.0001 ];
    Base_Dim = [1,1];
    %Cube points in the World Frame:
    side = 0.5;
    initial_corner = [0.5,0.5,5];
    rx = 0.1;
    ry = 0.1;
    rz = 0.1;
    cube_p = Cube_points(side,initial_corner,rx,ry,rz);  %8 rows for 8 points and 3 rows for x y z    
    %We need to define a "desired" position for the robot end-effector!
    angle_desired = [0*pi/180 , 0*pi/180 , 60*pi/180];
    qr = [Basecentre_xy(1),Basecentre_xy(2),0,0,0,0];
    theta_desired = [angle_desired,qr];
    %Desired Points in Camera frame
    Pc_desired = Point_in_CameraFrame(theta_desired,cube_p'); %[x;y;z] 
    %Points in image coordiates
    p_desired = Point_to_ImageCoord(cam,Pc_desired);
%     figure(200)
%     my_robot_plotter(Link_Length,angle_desired.*180/pi,qr,Base_Dim);
end

