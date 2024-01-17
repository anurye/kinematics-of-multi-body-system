function zero = Zero
%Zero.m: defines what numbers should be treated as zero. This is because
%may not get exactly zero when performing some computations: line
%determinant of the Jacobian. Any number under the zero defined here will
%be considered as zero
%
%   Parameter:
%             None
%
%   Returns:
%           zero: approximate definition of zero in this implementation
%

zero = 1e-6;

end

