function [R,t] = imgproc(filename1, filename2)

im1  = iread(filename1, 'mono', 'double');
im2  = iread(filename2, 'mono', 'double');

s1 = isurf(im1);
s2 = isurf(im2);
m = s1.match(s2);

camnew = CentralCamera('image', im1);

F = m.ransac(@fmatrix, 1e-4, 'verbose');
idisp({im1, im2})
m.inlier.subset(100).plot('g')

K = [1211.2959, 0, 657.15924;
    0, 1206.00512, 403.17667;
    0, 0, 1];

E = K'*F*K
sol = camnew.invE(E, [0,0,10]')
[R, t] = tr2rt(sol)

% 
% 
% camold = CentralCamera('image', im1);
% %camnew = CentralCamera('image', im2);
% 
% camold.invE(E)
% inv(camold.T) * cam2.T
% 
% 
% 
% m.show
% idisp({im1, im2})
% 
% m.inlier.plot('w')
% 
% [m,corresp] = s1.match(s2);
% 
% m2 = s1.match(s2, 'thresh', []);
% ihist(m2.distance, 'normcdf')
% 
% T1 = transl(-0.1, 0, 0) * troty(0.4);
% cam1 = CentralCamera('name', 'camera 1', 'default', ...
% 0.002, 'pose', T1)
% 
% T2 = transl(0.1, 0,0)*troty(-0.4);
% cam2 = CentralCamera('name', 'camera 2', 'default', ...
% 0.002, 'pose', T2);
% 
% %% RANSAC not performed on p1 and p2 - so we cannot have outliers
% F = fmatrix(p1, p2)
% rank(F)
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
