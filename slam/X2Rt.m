function [R,t,s,d]=X2Rt(X1,X2,scale)
% X2=s*R*X1+t
% X1, X2 in 3 by N matrix

if size(X1)~=size(X2) | size(X1,1)~=3 | size(X2,1)~=3
    error('X1 and X2 in 3*N');
end

N=size(X1,2);
m1=mean(X1,2); m2=mean(X2,2);
X1c=X1-m1(:,ones(1,N));
X2c=X2-m2(:,ones(1,N));

% scale factor
if scale
    s=sqrt(sum(sum(X2c.*X2c))/sum(sum(X1c.*X1c)));
else
    s=1;
end

M=X2c*X1c';
[u d v]=svd(M);
R=u*diag([1 1 det(u*v')])*v';

t=m2-s*R*m1;

d=X2-s*R*X1-t(:,ones(N,1));
%d=X2-X1;