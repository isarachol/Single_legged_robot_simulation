function xnext = numerical_check(x, tol)
%numerical_check compares values against a some numerical tolerance,
%checking if numerical problems are occurring. Returns 0 if any of the
%elements in x has abs(x(i))>tol or is NaN

% assume x is a vector, nx1.
xnext = x;

for i=1:size(x,1)
    if (abs(x(i)) > tol) || isnan(x(i))
        xnext = zeros(size(x));
        disp('Numerical issue detected');
    end
end

% return x; % occurs automatically

end