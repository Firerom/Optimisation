%% Part 1: Constraint v(s)<=b
%v(s)-b -> Matrix M1
syms m1_1 m1_2 m1_3 m1_4 m1_5 m1_6 m1_7 m1_8 m1_9 m1_10 m1_11 m1_12; 
syms m2_2 m2_3 m2_4 m2_5 m2_6 m2_7 m2_8 m2_9 m2_10 m2_11 m2_12; 
syms m3_3 m3_4 m3_5 m3_6 m3_7 m3_8 m3_9 m3_10 m3_11 m3_12; 
syms m4_4 m4_5 m4_6 m4_7 m4_8 m4_9 m4_10 m4_11 m4_12; 
syms m5_5 m5_6 m5_7 m5_8 m5_9 m5_10 m5_11 m5_12; 
syms m6_6 m6_7 m6_8 m6_9 m6_10 m6_11 m6_12; 
syms m7_7 m7_8 m7_9 m7_10 m7_11 m7_12; 
syms m8_8 m8_9 m8_10 m8_11 m8_12; 
syms m9_9 m9_10 m9_11 m9_12;
syms m10_10 m10_11 m10_12; 
syms m11_11 m11_12;
syms m12_12; 
 
syms x y theta theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9 theta10;
vec_coord=[1 x y theta theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9];
Matrix=[m1_1 	m1_2	m1_3	m1_4	m1_5	m1_6	m1_7	m1_8	m1_9	m1_10	m1_11	m1_12
        m1_2 	m2_2 	m2_3	m2_4	m2_5	m2_6	m2_7	m2_8	m2_9	m2_10	m2_11	m2_12	
        m1_3 	m2_3 	m3_3 	m3_4	m3_5	m3_6	m3_7	m3_8	m3_9	m3_10	m3_11	m3_12	
        m1_4 	m2_4 	m3_4 	m4_4 	m4_5	m4_6	m4_7	m4_8	m4_9	m4_10	m4_11	m4_12	
        m1_5 	m2_5 	m3_5 	m4_5 	m5_5 	m5_6	m5_7	m5_8	m5_9	m5_10	m5_11	m5_12	
        m1_6 	m2_6 	m3_6 	m4_6 	m5_6 	m6_6 	m6_7	m6_8	m6_9	m6_10	m6_11	m6_12	
        m1_7 	m2_7 	m3_7 	m4_7 	m5_7 	m6_7 	m7_7 	m7_8	m7_9	m7_10	m7_11	m7_12	
        m1_8 	m2_8 	m3_8 	m4_8 	m5_8 	m6_8 	m7_8 	m8_8 	m8_9	m8_10	m8_11	m8_12	
        m1_9 	m2_9 	m3_9 	m4_9 	m5_9 	m6_9 	m7_9 	m8_9 	m9_9 	m9_10	m9_11	m9_12	
        m1_10 	m2_10 	m3_10 	m4_10 	m5_10 	m6_10 	m7_10 	m8_10 	m9_10 	m10_10 	m10_11	m10_12	
        m1_11 	m2_11 	m3_11 	m4_11 	m5_11 	m6_11 	m7_11 	m8_11 	m9_11 	m10_11 	m11_11 	m11_12	
        m1_12 	m2_12 	m3_12 	m4_12 	m5_12 	m6_12 	m7_12 	m8_12 	m9_12 	m10_12 	m11_12 	m12_12 ]; 




syms c0 c1 c2 c3 c4 c5 c6 c7 c8 c9 c10;
syms u v fac2 fac3 fac4 fac5 fac6 fac7 fac8 fac9 fac10 K; 
grad=[c1+c4*y+c5*theta+2*c7*x c2+c4*x+c6*theta+2*c8*y c3+c5*x+c6*y+2*c9*theta];
sin_theta=theta-theta^3/fac3+theta^5/fac5-theta^7/fac7+theta^9/fac9;
cos_theta=1-theta^2/fac2+theta^4/fac4-theta^6/fac6;
sdot=[-v*sin_theta v*cos_theta -K*(theta-u)];

reponse=grad*sdot.'

mescoeffs_theta=coeffs(reponse,theta)

%terme independant de theta
mescoeffs_theta(1)
ok=coeffs(mescoeffs_theta(1),[y x]);
terme_independant=ok(1)
terme_x=ok(2)
terme_y=ok(3)

%terme dependant de theta
mescoeffs_theta(2)
ok=coeffs(mescoeffs_theta(2),[y x]);
terme_t=ok(1)
terme_xt=ok(2)
terme_yt=ok(3)

%terme dependant de theta2
mescoeffs_theta(3)
ok=coeffs(mescoeffs_theta(3),[y x]);
terme_t2=ok(1)
terme_xt2=ok(2)
terme_yt2=ok(3)

%terme dependant de theta3
mescoeffs_theta(4)
ok=coeffs(mescoeffs_theta(4),[y x]);
terme_t3=ok(1)
terme_xt3=ok(2)
terme_yt3=ok(3)

%terme dependant de theta4
mescoeffs_theta(5)
ok=coeffs(mescoeffs_theta(5),[y x]);
terme_t4=ok(1)
terme_xt4=ok(2)
terme_yt4=ok(3)


%terme dependant de theta5
mescoeffs_theta(6)
ok=coeffs(mescoeffs_theta(6),[y x]);
terme_t5=ok(1)
terme_xt5=ok(2)
terme_yt5=ok(3)


%terme dependant de theta6
mescoeffs_theta(7)
ok=coeffs(mescoeffs_theta(7),[y x]);
terme_t6=ok(1)
terme_xt6=ok(2)
terme_yt6=ok(3)

%terme dependant de theta7
mescoeffs_theta(8)
ok=coeffs(mescoeffs_theta(8),[y x]);
terme_t7=ok(1)
terme_xt7=ok(2)
terme_yt7=ok(3)

%terme dependant de theta8
mescoeffs_theta(9)
ok=coeffs(mescoeffs_theta(9),[y x]);
terme_t8=ok(1)

%terme dependant de theta9
mescoeffs_theta(10)
ok=coeffs(mescoeffs_theta(10),[y x]);
terme_t9=ok(1)
terme_xt9=ok(2)
terme_yt9=ok(3)

%terme dependant de theta10
mescoeffs_theta(11)
ok=coeffs(mescoeffs_theta(11),[y x]);
terme_t10=ok(1)

All_terme=[terme_independant
terme_x
terme_y
terme_xt
terme_xt2
terme_xt3
terme_xt4
terme_xt5
terme_xt6
terme_xt7
terme_xt9
terme_yt
terme_yt2
terme_yt3
terme_yt4
terme_yt5
terme_yt6
terme_yt7
terme_yt9
terme_t
terme_t2
terme_t3
terme_t4
terme_t5
terme_t6
terme_t7
terme_t8
terme_t9
terme_t10]

for i=1:length(All_terme)
    fprintf('%c\n',All_terme{i});
end
