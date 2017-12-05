%%
clear all 
fileID = fopen('myfile.txt','w');
compt=1;
for i=0:4
    puisx=0:i;
    puisy=0:i;
    puisz=0:i;
    for m=1:(i+1)
        for n=1:(i+1)
            for j=1:(i+1)
                total=puisx(j)+puisy(n)+puisz(m);
                if (total==i)
                fprintf(fileID,'%d %d %d \n',puisx(j),puisy(n),puisz(m));
                stock(compt,:)=[puisx(j),puisy(n),puisz(m)];
                compt=compt+1;
                end
                
            end
        end
    end
end
fclose(fileID);
%%
syms x y theta polyvec c0 c1 c2 c3 c4 c5 c6 c7 c8 c9 c9 c10 c11 c12 c13 c14 c15 c16 c17 
syms c18 c19 c20 c21 c22 c23 c24 c25 c26 c27 c28 c29  c30 c31 c32 c33 c34
syms s_dot v w K u
%polyvec=[x;y;theta];
ci=[c0 c1 c2 c3 c4 c5 c6 c7 c8 c9 c10 c11 c12 c13 c14 c15 c16 c17 ...
    c18 c19 c20 c21 c22 c23 c24 c25 c26 c27 c28 c29  c30 c31 c32 c33 c34];

for i=1:(compt-1)
        polyvec(i,1)=ci(i)*x^(stock(i,1))*y^(stock(i,2))*theta^(stock(i,3));
end

V=sum(polyvec);
V_diff_x=sum(diff(polyvec,x));
V_diff_y=sum(diff(polyvec,y));
V_diff_theta=sum(diff(polyvec,theta));


V_diff=[V_diff_x V_diff_y V_diff_theta];%vecteur ligne 
s_dot=[w;v;K*u];


V_dot=V_diff*s_dot;
mescoeffs=coeffs(V_dot,theta);
%% contrainte pour theta
syms x_o y_o
%terme indépendant de theta
terme_0_theta=sum(polyvec(find(diff(polyvec,theta)==0)));


vect_temp=polyvec(find(diff(polyvec,theta)~=0));
terme_1_theta=coeffs(sum(vect_temp(find(diff(vect_temp,theta,2)==0))),theta);
subs(subs(terme_1_theta,x,x_o),y,y_o);

vect_temp_2=polyvec(find(diff(polyvec,theta,2)~=0));
terme_2_theta=coeffs(sum(vect_temp_2(find(diff(vect_temp_2,theta,3)==0))),theta);
subs(subs(terme_2_theta,x,x_o),y,y_o);

vect_temp_3=polyvec(find(diff(polyvec,theta,3)~=0));
terme_3_theta=coeffs(sum(vect_temp_3(find(diff(vect_temp_3,theta,4)==0))),theta);
subs(subs(terme_3_theta,x,x_o),y,y_o);


vect_temp_4=polyvec(find(diff(polyvec,theta,4)~=0));
terme_4_theta=coeffs(sum(vect_temp_4(find(diff(vect_temp_4,theta,5)==0))),theta);
subs(subs(terme_4_theta,x,x_o),y,y_o);

%%

