function  p  = Point_to_ImageCoord(cam,Pc )
% Pc = Point wrt camera coordinate
% p = Image coordinate 
% f = Focal length
Pc = cam.K*Pc;
f = cam.K(1,1)/1000;
p = [ f * ( Pc(1,:)./ Pc(3,:));  f * ( Pc(2,:)./ Pc(3,:))];
end