function [cam_extr_stack,theta] = my_robot(arm_len,input_angle,base_lw,basecentre_xy,phi,th,psi)
    input_angle(4) = input_angle(4)+180;
    qr = [phi,th,psi];
    base_dim = [basecentre_xy,qr];
    xc = base_dim(1);
    yc = base_dim(2);
    length = base_lw(1);
    breadth = base_lw(2);
    %Base at origin%
    x = 0;
    y = 0;
    o = [x-length/2, y-breadth/2];
    corner_pts = [o(1), o(2), 0;
                  o(1)+length, o(2), 0;
                  o(1)+length, o(2)+breadth, 0;
                  o(1), o(2)+breadth, 0];    %centre is at 0,0,0
    mid = [length/2,0,0];  %Right Midpt
    mid2 = [-length/2,0,0]; %Left Midpt
    base_rot = [cosd(phi), -sind(phi), 0;
                sind(phi), cosd(phi),  0;
                0, 0, 1];
    rotated_base = base_rot*corner_pts.';
    mid = base_rot*mid.';
    mid2 = base_rot*mid2.';
    translated_base = [rotated_base(1,:)+xc; rotated_base(2,:)+yc; rotated_base(3,:)];
    translated_mid = [mid(1,:)+xc; mid(2,:)+yc; mid(3,:)]; %Right Link start position
    translated_mid2 = [mid2(1,:)+xc; mid2(2,:)+yc; mid2(3,:)]; %Left Link start position
    %Right link end positions
    x = [arm_len(1)*cosd(phi+input_angle(1)), arm_len(2)*cosd(phi+input_angle(1)+input_angle(2)),arm_len(3)*cosd(phi+input_angle(1)+input_angle(2)+input_angle(3))];
    start_links_x = [translated_mid(1)+0, translated_mid(1)+x(1), translated_mid(1)+x(1)+x(2), translated_mid(1)+x(1)+x(2)+x(3)];
    y = [arm_len(1)*sind(phi+input_angle(1)), arm_len(2)*sind(phi+input_angle(1)+input_angle(2)),arm_len(3)*sind(phi+input_angle(1)+input_angle(2)+input_angle(3))];
    start_links_y = [translated_mid(2)+0, translated_mid(2)+y(1), translated_mid(2)+y(1)+y(2), translated_mid(2)+y(1)+y(2)+y(3)];
    cam_xy_right = [start_links_x(4),start_links_y(4),0]; %Right camera position
    cam_rot_right = [cosd(phi+input_angle(1)+input_angle(2)+input_angle(3)), -sind(phi+input_angle(1)+input_angle(2)+input_angle(3)), 0;
                sind(phi+input_angle(1)+input_angle(2)+input_angle(3)), cosd(phi+input_angle(1)+input_angle(2)+input_angle(3)),  0;
                0, 0, 1];
    cam_extr_right = [cam_rot_right' -cam_rot_right'*cam_xy_right']; %camera position calculation [-R' -R'T]
    %Left Link end positions
    x2 = [arm_len(4)*cosd(phi+input_angle(4)),arm_len(5)*cosd(phi+input_angle(4)+input_angle(5)),arm_len(6)*cosd(phi+input_angle(4)+input_angle(5)+input_angle(6))];
    start_links_x2 = [translated_mid2(1)+0, translated_mid2(1)+x2(1), translated_mid2(1)+x2(1)+x2(2), translated_mid2(1)+x2(1)+x2(2)+x2(3)];
    y2 = [arm_len(4)*sind(phi+input_angle(4)),arm_len(5)*sind(phi+input_angle(4)+input_angle(5)),arm_len(6)*sind(phi+input_angle(4)+input_angle(5)+input_angle(6))];
    start_links_y2 = [translated_mid2(2)+0, translated_mid2(2)+y2(1), translated_mid2(2)+y2(1)+y2(2), translated_mid2(2)+y2(1)+y2(2)+y2(3)];
    cam_xy_left = [start_links_x2(4),start_links_y2(4),0];    %Left Camera position
    cam_rot_left = [cosd(phi+input_angle(4)+input_angle(5)+input_angle(6)), -sind(phi+input_angle(4)+input_angle(5)+input_angle(6)), 0;
                sind(phi+input_angle(4)+input_angle(5)+input_angle(6)), cosd(phi+input_angle(4)+input_angle(5)+input_angle(6)),  0;
                0, 0, 1];
    cam_extr_left = [cam_rot_left' -cam_rot_left'*cam_xy_left'];     %camera position calculation [-R' -R'T]
    cam_extr_stack = [cam_extr_right;cam_extr_left];                 %left and right camera positions stacked.
    theta = [input_angle,basecentre_xy,qr];
    figure(1)
    axis([-10 10 -10 10]);
    hold on;
    line([translated_base(1,:),translated_base(1,1)],[translated_base(2,:),translated_base(2,1)]);
    line(start_links_x(1:2),start_links_y(1:2));
    line(start_links_x(2:3),start_links_y(2:3));
    line(start_links_x(3:4),start_links_y(3:4));
    line(start_links_x2(1:2),start_links_y2(1:2));
    line(start_links_x2(2:3),start_links_y2(2:3));
    line(start_links_x2(3:4),start_links_y2(3:4));
    hold off;
end
