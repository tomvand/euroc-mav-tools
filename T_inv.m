function [ Tinv ] = T_inv( T )
%T_INV Invert transformation matrix

R = T(1:3,1:3);
t = T(1:3,4);

Tinv = NaN(4,4);
Tinv(1:3,1:3) = R';
Tinv(1:3,4) = -R'*t;
Tinv(4,:) = [0 0 0 1];

end

