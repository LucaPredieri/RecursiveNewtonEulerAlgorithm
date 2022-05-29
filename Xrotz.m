
function X = Xrotz( h )
% Xrotz  MM6 coordinate transform from Z-axis rotation.
% Xrotz(h) calculates the MM6 coordinate transform matrix (for motion
% vectors) induced by a rotation about the +Z axis by an angle h (in radians).
% Positive rotation is anticlockwise: +X axis rotates towards +Y axis.

c = cos(h);  s = sin(h);

X = [  c  s  0  0  0  0 ;
      -s  c  0  0  0  0 ;
       0  0  1  0  0  0 ;
       0  0  0  c  s  0 ;
       0  0  0 -s  c  0 ;
       0  0  0  0  0  1
    ];