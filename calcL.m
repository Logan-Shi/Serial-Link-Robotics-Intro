clear all;clc;addpath(genpath('.'));
    Lxx = 12993691.651  ;Lxy = -3104762.061  ;Lxz = 4269360.229;
    Lyx = -3104762.061  ;Lyy = 21553881.669  ;Lyz = -586334.637;
    Lzx = 4269360.229   ;Lzy = -586334.637   ;Lzz = 20264292.879;

Ii = [Lxx,Lxy,Lxz;
    Lyx,Lyy,Lyz;
    Lzx,Lzy,Lzz];

I(:,:,1) = Ii;

Lxx = 2593576.319	;Lxy = 410236.550	;Lxz = -170604.112;
	Lyx = 410236.550	;Lyy = 16923536.866	;Lyz = -15485.176;
	Lzx = -170604.112	;Lzy = -15485.176	;Lzz = 18214382.975;

Ii = [Lxx,Lxy,Lxz;
    Lyx,Lyy,Lyz;
    Lzx,Lzy,Lzz];

I(:,:,2) = Ii;

Lxx = 36532423.796  ;Lxy = 126134.093    ;Lxz = 210796.638;
    Lyx = 126134.093    ;Lyy = 37991186.833  ;Lyz = 434883.263;
    Lzx = 210796.638    ;Lzy = 434883.263    ;Lzz = 4847084.472;

Ii = [Lxx,Lxy,Lxz;
    Lyx,Lyy,Lyz;
    Lzx,Lzy,Lzz];

I(:,:,3) = Ii;

    Lxx = 620457.244    ;Lxy = 0.275 ;Lxz = 0.910;
    Lyx = 0.275 ;Lyy = 453973.560    ;Lyz = -3704.293;
    Lzx = 0.910 ;Lzy = -3704.293 ;Lzz = 413711.800;

Ii = [Lxx,Lxy,Lxz;
    Lyx,Lyy,Lyz;
    Lzx,Lzy,Lzz];

I(:,:,4) = Ii;

    Lxx = 81899.101 ;Lxy = -0.046    ;Lxz = 0.946;
    Lyx = -0.046    ;Lyy = 78100.979; Lyz = -384.257;
    Lzx = 0.946 ;Lzy = -384.257  ;Lzz = 81099.778;


Ii = [Lxx,Lxy,Lxz;
    Lyx,Lyy,Lyz;
    Lzx,Lzy,Lzz];

I(:,:,5) = Ii;

    Lxx = 24948.577; Lxy = 0.001 ;Lxz = 3.774;
    Lyx = 0.001 ;Lyy = 22332.066 ;Lyz = -0.003;
    Lzx = 3.774 ;Lzy = -0.003    ;Lzz = 21262.320;

Ii = [Lxx,Lxy,Lxz;
    Lyx,Lyy,Lyz;
    Lzx,Lzy,Lzz];

I(:,:,6) = Ii;

save I.mat
rmpath(genpath('.'))