x = interval([-2;-1],[2;2]);
y = interval([3;3],[4;4]);
X = mptPolytope(x);
Y = mptPolytope(y);
S(1) = X;
S(2) = Y;
plot(or(X,Y))
% x = sdpvar;
% S(1) = [ 1 <= x <= 2 ];
% S(2) = [ 1.5 <= x <= 6 ];
% Union(S)