
%Diagonally dominant
%xII>=sum(xIJ) with i~=j
Taille=12
for i=1:Taille
    fprintf('@constraint(m,-M[%d,%d]>=',i,i);
    for j=1:Taille     
        if (j==Taille)&&(i~=j)
            fprintf('abs_(M[%d,%d])',i,j);
        elseif (j==Taille-1)&&(i==Taille)
            fprintf('abs_(M[%d,%d])',i,j);
        else
            if i~=j
            fprintf('abs_(M[%d,%d])+',i,j);
            end
        end
    end
    fprintf(')\n')
end    
