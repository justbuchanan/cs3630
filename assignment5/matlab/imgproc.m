im1  = iread('data/pic-1427427441.jpg', 'mono', 'double');
im2  = iread('data/pic-1427427439.jpg', 'mono', 'double');

s1 = isurf(im1);
s2 = isurf(im2);
m = s1.match(s2);

F = m.ransac(@fmatrix, 1e-4, 'verbose');
idisp({im1, im2})
m.inlier.subset(100).plot('g')

E = K'*F*K;
[R, t] = tr2rt(E)

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
