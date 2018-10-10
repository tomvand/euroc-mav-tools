function [ R ] = all_q2R( q )
%ALL_Q2C Transform all quaternions in q (4xN) to rotation matrices R
%(3x3xN)
%
% Note: this transforms quaternion q_RS into matrix R_RS, so the inverse
% of q_q2C which produces R_SR!!!

N = size(q, 2);
R = NaN(3, 3, N);
for i = 1:N
    R(:,:,i) = q_q2C(q(:,i))'; % transposed to get R_RS, see note above
end

end

