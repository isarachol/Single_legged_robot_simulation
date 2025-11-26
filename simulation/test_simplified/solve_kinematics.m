function T = solve_kinematics(q, joint_to_com, rot)
% given configuration, geometry, and relative rotations
% assumption1 every joint is rotated about relative x axis
% rot helps retate each link to the correct direction for assumption1 to be
% true

zero_vec = [0;0;0];
N = numel(q);
T = cell(N,1);
for i=1:N
    % link
    T{i} = Tx(q(i), joint_to_com(:,i));

    % joints has no length
    if rot(1,i) ~= 0 
        T{i} = T{i} * Tx(rot(1,i),zero_vec);
    end
    if rot(2,i) ~= 0
        T{i} = T{i} * Ty(rot(2,i),zero_vec);
    end
    if rot(3,i) ~= 0
        T{i} = T{i} * Tz(rot(3,i),zero_vec);
    end

    % extrinsic transformations
    if i>1
        T{i} = T{i-1} * T{i};
    end
end

end