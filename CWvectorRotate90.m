function [x2,y2] = CWvectorRotate90(x0,y0,x1,y1,offset)

v1 = [x1-x0;y1-y0];

v1norm = v1/norm(v1);

v1offsetlen = v1norm*offset;

%rotate it

neg90rot = [cos(-pi/2),-sin(-pi/2); sin(-pi/2), cos(-pi/2)];

v1offsetlenrot = neg90rot*v1offsetlen;

x2 = x0 + v1offsetlenrot(1);
y2 = y0 + v1offsetlenrot(2);

end