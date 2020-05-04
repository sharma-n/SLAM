function [zNorm] = normalize_all_angles(z)
% Go over all angles and normalize them

for(i=1:length(z))
   z(i) = normalize_angle(z(i));
endfor
zNorm = z;
