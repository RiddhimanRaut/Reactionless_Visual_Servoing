function imgxy = point_to_image_coord(extr_matrix,P_world)
    %P_world = 3d world coordinate of feature pt 3x1
    P_world = [P_world;1];
    cam.K=[800 0 320; 0 800 240; 0 0 1];
    img = cam.K*extr_matrix*P_world;
    imgx = img(1,:)/img(3,:);
    imgy = img(2,:)/img(3,:);
    imgxy = [imgx,imgy];
end