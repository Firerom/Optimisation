clear all; close all;clc;
Dossier='Result_Q2'

u_data=[0 pi];
file_data={'obs_behind','obs_behind_side'};

for i=1:length(file_data)
    for l=1:length(u_data)
        uvalue=u_data(l);
        if abs(uvalue)<10^-5
            u='0';
        else
            u='pi';
        end
        file=file_data{i}
        C=load([Dossier '/Q2_' u '_' file '.txt'])'
        plot_V(C,u,file)
    end
end