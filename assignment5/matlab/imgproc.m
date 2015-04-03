function [R,t] = imgproc(filename1, filename2)

im1  = iread(filename1, 'mono', 'double');
im2  = iread(filename2, 'mono', 'double');

s1 = isurf(im1);
s2 = isurf(im2);
m = s1.match(s2);

% camnew = CentralCamera('image', im1);

% if length(m) < 7
%     R = [0 0 0; 0 0 0; 0 0 0];
%     t = [0 0]';
% end


try
    F = m.ransac(@fmatrix, 1e-4, 'verbose');
catch
    R = [0 0 0; 0 0 0; 0 0 0];
    t = [0 0]';
    return;
end

idisp({im1, im2})
% m.inlier.subset(100).plot('g')

K = [1211.2959, 0, 657.15924;
    0, 1206.00512, 403.17667;
    0, 0, 1];

E = K'*F*K
% sol = camnew.invE(E, [0,0,10]')
% [R, t] = tr2rt(sol)



[U,S,V] = svd(E);

% Ma etal solution, p116, p120-122
% Fig 5.2 (p113), is wrong, (R,t) is from camera 2 to 1
if det(V) < 0
    V = -V;
    S = -S;
end
if det(U) < 0
    U = -U;
    S = -S;
end
R1 = U*rotz(pi/2)'*V';
R2 = U*rotz(-pi/2)'*V';
t1 = vex(U*rotz(pi/2)*S*U');
t2 = vex(U*rotz(-pi/2)*S*U');
% invert (R,t) so its from camera 1 to 2
s(:,:,1) = inv( [R1 t1; 0 0 0 1] );
s(:,:,2) = inv( [R2 t2; 0 0 0 1] );

% [R,t] = R1, t1
[R, t] = tr2rt(s(:,:,2));
t = t(1:2); % FIXME: which 2 of the 3 components in t do we want to extract?
