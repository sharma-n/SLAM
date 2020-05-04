p1 = [276; 30; pi/2];
p2 = [120; 66; pi/2];
fprintf('Check if t2v->v2t gives back same vector: %s.\n', mat2str(t2v(v2t(p1))==p1))

# To go from p1 to p2: movement = p2 - p1
M = v2t(p2-p1);

# robot pose p1, landmark observation z
p1 = [1; 1; 1];
z = [2; 0; 1]; # z = [2; 0] wwith 1 added to convert to mogenous oordinates.
abs_landmark = p1+z