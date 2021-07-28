function Pc = Point_in_CameraFrame(Combined_theta , P )

    % P = Point in world frame
    % O = Camera co-ordinates
    theta = Combined_theta(1:3);
    th = Combined_theta(7);
    xc = Combined_theta(4);
    yc = Combined_theta(5);
    len = 1;
    l=[1,1,1];
    %-------Rotation Matrix for base-------------%
    r = [cosd(th),-sind(th),0;
         sind(th), cosd(th),0;
               0 ,        0,1];% Base rotation matrix
    %-------Finding start of link--------------------%
    mid = r*[len/2,0,0].'; 
    start_link = [ mid(1)+xc,mid(2)+yc ];
    
    R = [  cos( th+theta(1) + theta(2) + theta(3) ) , - sin( th+theta(1) + theta(2) + theta(3) ) , 0 ;
           sin( th+theta(1) + theta(2) + theta(3) ) ,   cos( th+theta(1) + theta(2) + theta(3) ) , 0 ;
                                                        0 ,                                                0 , 1 ];
    O = [start_link(1)+(l(1)*cos(theta(1)+th))+(l(2)*cos(theta(1)+th+theta(2)))+(l(3)*cos(theta(1)+th+theta(2)+theta(3)));
         start_link(2)+(l(1)*sin(theta(1)+th))+(l(2)*sin(theta(1)+th+theta(2)))+(l(3)*sin(theta(1)+th+theta(2)+theta(3)));
                                                                                                                                  0 ];                              
    Pc = transpose(R) * (P - O);

end