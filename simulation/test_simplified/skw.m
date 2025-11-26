function S = skw(w)
%skw Form the skew-symmetric matrix S based on a 3x1 vector (hint, this is
%usually angular velocity)
%   
%   input: w = 3x1 vector
%
%   output: S = 3x3 matrix with entries as specified in this file. See, for
%   example, Siciliano eqn. 3.9

S = [0, -w(3), w(2);
    w(3), 0, -w(1);
    -w(2), w(1), 0];

end