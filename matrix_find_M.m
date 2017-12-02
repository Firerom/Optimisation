%%
clear all
syms x y theta poly mono 
syms m1_1 m1_2 m1_3 m1_4 m1_5 m1_6 m1_7 m1_8 
syms m2_2 m2_3 m2_4 m2_5 m2_6 m2_7 m2_8 
syms m3_3 m3_4 m3_5 m3_6 m3_7 m3_8 
syms m4_4 m4_5 m4_6 m4_7 m4_8 
syms m5_5 m5_6 m5_7 m5_8 
syms m6_6 m6_7 m6_8 
syms m7_7 m7_8 
syms m8_8 

poly=[1 x y theta x^2 y^2 theta^2 x*y x*theta theta*y x^3 y^3 x*y^2 x*theta^2 y*x^2 y*theta^2 theta^3 theta*x^2 theta*y^2 x*y*theta x^4 y^4 theta^4 x^2*y^2 x^2*theta^2 x^2*y*theta y^2*theta^2 y^2*x*theta theta^2*y*x x^3*y x^3*theta y^3*x y^3*theta theta^3*x theta^3*y];
size(poly)
mono=[1 x y theta x*y x^2 y^2 theta^2];

  M=[   m1_1 	m1_2	m1_3	m1_4	m1_5	m1_6	m1_7	m1_8;	
     m1_2 	m2_2 	m2_3	m2_4	m2_5	m2_6	m2_7	m2_8	;
     m1_3 	m2_3 	m3_3 	m3_4	m3_5	m3_6	m3_7	m3_8	;
     m1_4 	m2_4 	m3_4 	m4_4 	m4_5	m4_6	m4_7	m4_8	;
     m1_5 	m2_5 	m3_5 	m4_5 	m5_5 	m5_6	m5_7	m5_8	;
     m1_6 	m2_6 	m3_6 	m4_6 	m5_6 	m6_6 	m6_7	m6_8	;
     m1_7 	m2_7 	m3_7 	m4_7 	m5_7 	m6_7 	m7_7 	m7_8	;
     m1_8 	m2_8 	m3_8 	m4_8 	m5_8 	m6_8 	m7_8 	m8_8];
 syms contribution a 
contribution=[a a a a a a a a a a a a a a a a a a a a a a a a a a a a a  a a a a a a ];
size(contribution)
for i=1:8
    for j=1:8
        for m=1:max(size(poly))
            if(mono(i)*mono(j)==poly(m))%on sait que l'element de matrice doit apporter une contribution pour l'égalité avec poly(m)
               
                contribution(m)=contribution(m)+M(i,j);
            %elseif( m==max(size(poly)) && mono(i)*mono(j)~=poly(m))
            %    contribution(max(size(contribution)))=contribution(max(size(contribution)))+M(i,j);
                
               
            end
        end
    end
end

total=[poly.' subs(contribution,a,0).']
%subs(contribution,a,0).'