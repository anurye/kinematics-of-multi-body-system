function rms = MyRMS(x1, x2)
% MyRMS: computes the root mean squared error between x1 and x2
%
%   Parameters:
%            x1: array 1
%            x2: array 2
%
%   Returns:
%          rms: rms of the difference between x1 and x2
%

% Validate if x1 and x2 have equal lengths
if ~ length(x1) == length(x2)
    error("MyRMS: the length of the two arrays (x1 and x2) must be the same")
end

rms = sqrt(sum((x1 - x2).^2)/length(x1));

end