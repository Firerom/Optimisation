%M[i,j]+M[i,j]+...=0




Constraint{1}='c2*v + K*c3*u'

                 c4*v + K*c5*u
               2*c8*v + K*c6*u
               - K*c5 - 2*c7*v
                  -(c4*v)/fac2
                 (2*c7*v)/fac3
                   (c4*v)/fac4
                -(2*c7*v)/fac5
                  -(c4*v)/fac6
                 (2*c7*v)/fac7
                -(2*c7*v)/fac9
                 - K*c6 - c4*v
                -(2*c8*v)/fac2
                   (c4*v)/fac3
                 (2*c8*v)/fac4
                  -(c4*v)/fac5
                -(2*c8*v)/fac6
                   (c4*v)/fac7
                  -(c4*v)/fac9
 c6*v - c1*v - K*c3 + 2*K*c9*u
 - 2*K*c9 - c5*v - (c2*v)/fac2
     (c1*v)/fac3 - (c6*v)/fac2
     (c2*v)/fac4 + (c5*v)/fac3
     (c6*v)/fac4 - (c1*v)/fac5
   - (c2*v)/fac6 - (c5*v)/fac5
     (c1*v)/fac7 - (c6*v)/fac6
                   (c5*v)/fac7
                  -(c1*v)/fac9
                  -(c5*v)/fac9






Taille=12;
Sum=20;
p=9;
for Sum=16:24
    p=p+1;
    fprintf('#theta%d:0\n@constraint(m,',p);
for i=Sum-Taille:1:Taille
    if i~=Taille
        if i<Sum-i
            fprintf('M[%d,%d]+',i,Sum-i)
        else
            fprintf('M[%d,%d]+',Sum-i,i)
        end
    else
        if i<Sum-i
            fprintf('M[%d,%d]==0',i,Sum-i)
        else
            fprintf('M[%d,%d]==0',Sum-i,i)
        end
    end
end
fprintf(')\n')
end