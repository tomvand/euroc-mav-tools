function [ T ] = all_pC2T( p, C )
%ALL_PQ2T Merge all position and rotation vectors/matrices into Transformation
%matrices

N = size(p, 2);
T = NaN(4, 4, N);
for i=1:N
    T(1:3,1:3,i) = C(:,:,i);
    T(1:3,4,i) = p(:,i);
    T(4,:,i) = [0 0 0 1];
end

end

