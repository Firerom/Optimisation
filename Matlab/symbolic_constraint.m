%% Part 1: Constraint v(s)<=b
%v(s)-b -> Matrix M1
syms m11 m12 m13 m14 m22 m23 m24 m33 m34 m44;
syms x y theta;
vec_coord=[1 x y theta];
Matrix=[m11 m12 m13 m14; m12 m22 m23 m24; m13 m23 m33 m34; m14 m24 m34 m44];
V_b=vec_coord*Matrix*vec_coord.'
%coeffs(vec_coord*Matrix*vec_coord', [1 x y theta])
simplify(subs(V_b,[y],[0]))