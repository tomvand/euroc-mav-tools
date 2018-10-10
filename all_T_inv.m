function [ Tinv ] = all_T_inv( T )
%ALL_T_INV Invert all transformation matrices

N = size(T, 3);
Tinv = NaN(4, 4, N);
for i=1:N
    Tinv(:,:,i) = T_inv(T(:,:,i));
end

end

