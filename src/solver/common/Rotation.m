function R = Rotation(fi)
% Rotation.m: computes the directional cosine matrix for a given angle fi
%   Parameter: 
%             fi: rotation angle in radian
%
%   Returns: directional cosine matrix R
%

R = [cos(fi) -sin(fi);
     sin(fi) cos(fi)];

end