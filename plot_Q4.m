

clear all; close all;clc;
Dossier='Result_Q4_LP';

u_data=[0 pi];
file_data={'obs_behind','obs_behind_side'};
h=[-9999 -999999];
for k=1:length(h)
for i=1:length(file_data)
    for l=1:length(u_data)
        uvalue=u_data(l);
        if abs(uvalue)<10^-5
            u='0';
        else
            u='pi';
        end
        file=file_data{i}
        C=load([Dossier '/Q4_' u '_' file '_' int2str(h(k)) '.txt']).'
        plot_V_4(C,u,file)
    end
end
end