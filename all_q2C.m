function [ C ] = all_q2C( q )
%ALL_Q2C Transform all quaternions in q (4xN) to rotation matrices C
%(3x3xN)

N = size(q, 2);
C = NaN(3, 3, N);
for i = 1:N
    C(:,:,i) = q_q2C(q(:,i));
end

end

