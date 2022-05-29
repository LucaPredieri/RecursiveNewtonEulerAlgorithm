
function X = Xtrans( r )
% Xtrans  MM6 coordinate transform from 3D translation vector.
% Xtrans(r) calculates the MM6 coordinate transform matrix (for motion
% vectors) induced by a shift of origin specified by the 3D vector r, which
% contains the x, y and z coordinates of the new location of the origin
% relative to the old.

X = [  1     0     0    0  0  0 ;
       0     1     0    0  0  0 ;
       0     0     1    0  0  0 ;
       0     r(3) -r(2) 1  0  0 ;
      -r(3)  0     r(1) 0  1  0 ;
       r(2) -r(1)  0    0  0  1
    ];