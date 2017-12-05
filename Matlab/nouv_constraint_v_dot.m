%M[i,j]+M[i,j]+...=0




Constraint{1}='c2*v + K*c3*u';
Constraint{2}='c4*v + K*c5*u';
Constraint{3}='2*c8*v + K*c6*u';
Constraint{4}='- K*c5 - 2*c7*v';
Constraint{5}='-(c4*v)/2';
Constraint{6}='(2*c7*v)/6';
Constraint{7}='(c4*v)/24';
Constraint{8}='-(2*c7*v)/120';
Constraint{9}='-(c4*v)/720';
Constraint{10}='(2*c7*v)/5040';
Constraint{11}='-(2*c7*v)/362880';
Constraint{12}='- K*c6 - c4*v';
Constraint{13}='-(2*c8*v)/2';
Constraint{14}='(c4*v)/6';
Constraint{15}='(2*c8*v)/24';
Constraint{16}='-(c4*v)/120';
Constraint{17}='-(2*c8*v)/720';
Constraint{18}='(c4*v)/5040';
Constraint{19}='-(c4*v)/362880';
Constraint{20}='c6*v - c1*v - K*c3 + 2*K*c9*u';
Constraint{21}='- 2*K*c9 - c5*v - (c2*v)/2';
Constraint{22}='(c1*v)/6 - (c6*v)/2';
Constraint{23}='(c2*v)/24 + (c5*v)/6';
Constraint{24}='(c6*v)/24 - (c1*v)/120';
Constraint{25}='- (c2*v)/720 - (c5*v)/120';
Constraint{26}='(c1*v)/5040 - (c6*v)/720';
Constraint{27}='(c5*v)/5040';
Constraint{28}='-(c1*v)/362880';
Constraint{29}='-(c5*v)/362880';

Element{1}='M[1,1]';
Element{2}='2*M[1,2]';
Element{3}='2*M[1,3]';
%Element{4}='2*M[2,4]';
for l=0:1:6
    index=round(4)+l;
    Element{index}=['2*M[2,' int2str(4+l) ']'];
end
Element{11}='2*M[2,12]';
for l=0:1:6
    index=round(12)+l;
    Element{index}=['2*M[3,' int2str(4+l) ']'];
end
Element{19}='2*M[3,12]';
Element{20}='2*M[1,4]';
Element{21}='2*M[1,5]+M[4,4]';
Element{22}='2*M[1,6]+M[4,5]+M[4,5]';
Element{23}='2*M[1,7]+M[4,6]+M[5,5]+M[4,6]';
Element{24}='2*M[1,8]+M[4,7]+M[5,6]+M[5,6]+M[4,7]';
Element{25}='2*M[1,9]+M[4,8]+M[5,7]+M[6,6]+M[5,7]+M[4,8]';
Element{26}='2*M[1,10]+M[4,9]+M[5,8]+M[6,7]+M[6,7]+M[5,8]+M[4,9]';
Element{27}='2*M[1,11]+M[4,10]+M[5,9]+M[6,8]+M[7,7]+M[6,8]+M[5,9]+M[4,10]';
Element{28}='2*M[1,12]+M[4,11]+M[5,10]+M[6,9]+M[7,8]+M[7,8]+M[6,9]+M[5,10]+M[4,11]';
Element{29}='M[4,12]+M[5,11]+M[6,10]+M[7,9]+M[8,8]+M[7,9]+M[6,10]+M[5,11]+M[4,12]';

comment={'1','x','y','xt','xt2','xt3','xt4','xt5','xt6','xt7','xt9','yt','yt2','yt3','yt4','yt5','yt6','yt7','yt9',...
    't','t1','t2','t3','t4','t5','t6','t7','t8','t9','t10'};
for l=1:29
   fprintf('# %s\n@constraint(m,%s==%s)\n',comment{l},Constraint{l},Element{l}) 
end


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

for Sum=8:16
    p=p+1;
    %fprintf('#theta%d:0\n@constraint(m,',p);
    fprintf('2*M[1,%d]+',5+Sum-8);
for i=4:1:Sum-4
    if i~=(Sum-4)
        if i<Sum-i
            fprintf('M[%d,%d]+',i,Sum-i)
        else
            fprintf('M[%d,%d]+',Sum-i,i)
        end
    else
        if i<Sum-i
            fprintf('M[%d,%d]',i,Sum-i)
        else
            fprintf('M[%d,%d]',Sum-i,i)
        end
    end
end
fprintf('\n')
end


for Sum=8:16
    p=p+1;
    %fprintf('#theta%d:0\n@constraint(m,',p);
    fprintf('M[1,%d]+M[%d,1]+',5+Sum-8,5+Sum-8);
for i=4:1:Sum-4
    if i~=(Sum-4)
        fprintf('M[%d,%d]+',i,Sum-i)  
    else
        fprintf('M[%d,%d]',i,Sum-i)
    end
end
fprintf('\n')
end

Taille=12;
Sum=20;
p=9;
for Sum=16:24
    p=p+1;
    fprintf('#theta%d:0\n@constraint(m,',p);
for i=Sum-Taille:1:Taille
    if i~=Taille
        fprintf('M[%d,%d]+',i,Sum-i)
    else
        fprintf('M[%d,%d]==0',i,Sum-i)
    end
end
fprintf(')\n')
end
fprintf('\n\n')
for l=1:12
for i=1:11
    if i>=l
        valeur=i+1;
    else
        valeur=i;
    end
    fprintf('@constraint(m,aux[%d,%d]>=M[%d,%d]\n',l,i,l,valeur);
    fprintf('@constraint(m,aux[%d,%d]>=-M[%d,%d]\n',l,i,l,valeur);    
end
fprintf('\n')
end