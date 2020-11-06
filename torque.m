clear all;close all;clc;addpath(genpath('.'));
% 待定参数
load I.mat
scale = 1.5;
l1 = 0.45/scale;
l2 = 1.0/scale;
l3 = 1.31228/scale;
l4 = 0.28445/scale;
l5 = 0.4/scale;% 连杆长度mm
m = [205.811,102.832,267.675,25.069,10.895,3.354]; % 质量
r = [-209.222,130.066,-63.899;
    -546.723,46.512,-305.326;
    -1.664,73.292,305.529;
    0,-171.963,-84.563;
    0,83.811,14.196;
    0.011,82,-128.852;
    ]; % 质心坐标 mm

% 机器人初始化
a(1) = l5;  alpha(1) = -pi/2; d(1) = l1; theta(1) = 0;
a(2) = l2;  alpha(2) = 0;     d(2) = 0;  theta(2) = 0;
a(3) = 0;   alpha(3) = -pi/2; d(3) = 0;  theta(3) = 0;
a(4) = 0;   alpha(4) = pi/2;  d(4) = l3;  theta(4) = 0;
a(5) = 0;   alpha(5) = -pi/2; d(5) = 0;  theta(5) = 0;
a(6) = 0;   alpha(6) = 0;     d(6) = l4; theta(6) = 0;

for i = 1:6
    L(i) = Link([theta(i),d(i),a(i),alpha(i),0]);
    L(i).m = m(i)/(scale^3);
    L(i).r = r(i,:)/1000/scale;
    L(i).I = I(:,:,i)/(10^6)/(scale^5);
    L(i).Jm = 0;
end
L(1).qlim=[-pi,pi];
L(2).qlim=[-71.48/180*pi,73.52/180*pi];
L(3).qlim=[-204.22/180*pi,65.78/180*pi];
L(4).qlim=[-pi,pi];
L(5).qlim=[-110/180*pi,110/180*pi];
L(6).qlim=[-pi,pi];

R = SerialLink(L,'name','RRRRRR');
R.offset = [0 -pi/2 0 0 0 0];
% figure()
% R.teach()

%% 正运动学求位形空间
% sample = 2000;
% qs = zeros(6,sample);
% ps = zeros(3,sample);
% length_sample = zeros(1,sample);
% height_sample = zeros(1,sample);
% for j = 1:sample
%     for i = 2:6
%         qs(i,j) = unifrnd(R.qlim(i,1),R.qlim(i,2));
%     end
%     Tsample = R.fkine(qs(:,j));
%     ps(:,j) = transl(Tsample);
%     length_sample(j) = norm(ps(1:2,j));
%     height_sample(j) = norm(ps(3,j));
% end
% [maxlength,max_id] = max(length_sample);
% [maxheight,max_id_h] = max(height_sample);
% figure()
% scatter3(0,0,0,'r','filled')
% hold on
% scatter3(ps(1,max_id),ps(2,max_id),ps(3,max_id),'g','filled')
% scatter3(ps(1,max_id_h),ps(2,max_id_h),ps(3,max_id_h),'k','filled')
% scatter3(ps(1,:),ps(2,:),ps(3,:),'b')
% title(['horizontal reachability max at: ' num2str(maxlength) ...
%         '  vertical reachability max at: ' num2str(maxheight)])
% legend('origin','horizontal reachable point','vertical reachable point','reachable points');
% xlabel('x');ylabel('y');zlabel('z');
% axis equal
% figure()
% R.teach(qs(:,max_id)')

%% DH参数图
% q = [0,0,0,0,0,0];
% list = 1;
% for i = 1:6
%     Ptmp = R.A(list,q).T;
%     trplot(Ptmp)
%     hold on
%     if i ~=6
%         list = [list,i+1];
%     end
% end
% R.teach(q)

%% C
% qdmax = [pi,pi,pi,2*pi,2*pi,4*pi];
% q = [0,0,0,0,0,0];
% C = R.coriolis(q,qdmax);
% tauc = C*qdmax';

%% 正运动学求弹簧劲度系数
% q2 = linspace(R.qlim(2,1),R.qlim(2,2),100);
% L0 = linspace(0.28,0.46,8);
% Kspring = linspace(1e4,5e4,8);
% ktau = zeros(3,length(q2));
% dist = zeros(1,length(q2));
% g2 = zeros(1,length(q2));
% for k = 1:length(L0)
%     for j = 1:length(Kspring)
%         for i = 1:length(q2)
%             q = [0,q2(i),-pi/2,0,0,0];
%             P1 = R.A([1,2],q).T*transl(-1.080/scale,-0.24/scale,-0.360/scale);
%             P2 = R.A(1,q).T*transl(-0.667/scale,0.035/scale,-0.359/scale);
%             P3 = R.A([1,2],q).T*transl(-1/scale,0,-0.360/scale);
%             a = transl(P1)-transl(P2);
%             dist(i) = norm(a);
%             r = transl(P3)-transl(P1);
%             ktau(:,i) = Kspring(j)*cross((a-L0(k)*a/norm(a)),r);
%             g = R.gravload([0 q2(i) -pi/2 0 0 0]);
%             g2(i) = g(2);
%             %     trplot(P1)
%             %     hold on
%             %     trplot(P2)
%             %     trplot(P3)
%             %     R.teach(q)
%         end
%         net(j,k) = norm(g2+ktau(2,:),inf);
%     end
% end
% minkl = min(min(net));
% [ki,li] = find(net==minkl);
% figure()
% surf(L0,Kspring,net)
% title('spring design')
% ylabel('K,N/m')
% xlabel('L0,m')
% zlabel('max net torque,Nm')
% grid on
% 
% for i = 1:length(q2)
%     q = [0,q2(i),-pi/2,0,0,0];
%     P1 = R.A([1,2],q).T*transl(-1.080/scale,-0.24/scale,-0.360/scale);
%     P2 = R.A(1,q).T*transl(-0.667/scale,0.035/scale,-0.359/scale);
%     P3 = R.A([1,2],q).T*transl(-1/scale,0,-0.360/scale);
%     a = transl(P1)-transl(P2);
%     dist(i) = norm(a);
%     r = transl(P3)-transl(P1);
%     ktau(:,i) = Kspring(ki)*cross((a-L0(li)*a/norm(a)),r);
%     g = R.gravload([0 q2(i) -pi/2 0 0 0]);
%     g2(i) = g(2);
%     %     trplot(P1)
%     %     hold on
%     %     trplot(P2)
%     %     trplot(P3)
%     %     R.teach(q)
% end
% 
% figure()
% plot(q2,dist)
% title('q2-spring length')
% xlabel('q2,rad')
% ylabel('spring length,m')
% grid on
% 
% figure()
% plot(q2,ktau(2,:))
% hold on
% plot(q2, g2);
% plot(q2, g2+ktau(2,:));
% legend('spring','grav','net')
% title('load on joint2')
% xlabel('q2,rad')
% ylabel('torque,Nm')
% grid on

%% 逆运动学
n = 5;
m = 5;
count = 1;
signal = 1;
P = [];
for i = 1:n
    for j = 1:m
        P(2,count) = signal*(j-1)+(n-1)*(1-signal)/2;
        P(3,count) = i;
        P(1,count) = 1;
        count = count + 1;
    end
    signal = -signal;
end
P = [[0;0;0],P];
P = P/5;
P(3,:) = P(3,:)-1;
tspan = 0:0.1:8;
q = zeros(length(tspan),6);
[pt,vt,at,Jt] = BSplineC(P,4,tspan,[20,30,20],0,0);

T0 = R.fkine([0,0,0,0,0,0]).T;
for i = 1:length(tspan)
    Ti = transl(pt(:,i));
    if i == 1
        q(i,:) = R.ikine(Ti*T0,'q0',[0,0,0,0,0,0]);
    else
        q(i,:) = R.ikine(Ti*T0,'q0',q(i-1,:));
    end
end

% q1 = pi/20*[0 15/2/1.6 -17/1.5/1.4 0 -20/1.8 0];
% q2 = pi/20*[20/1.5/1.4 -5/2/1.6 3/1.5/1.4 40/1.55 20/1.8 80/1.55];
% T1=R.fkine(q1i);
% T2=R.fkine(q2i);  %两个位姿
% q1=R.ikine(T1,'q0',q1i);
% q2=R.ikine(T2,'q0',q2i);            %求逆
% R.teach(q2)

% T2 = transl(0.4,-0.2,0)*trotx(pi/2);
% q1 = R.ikine(T1);
% q2 = R.ikine(T2);

%% 轨迹插值
% t=0:0.01:1.0;
% len = length(t);    %运动2s，采样时间0.05s
% qd1 = zeros(6,1);
% qd2 = [pi/1.3,-pi/1.1,pi/1.1,2*pi,2*pi,4*pi];
% % joint space
% [q,qd,qdd] = jtraj(q1,q2,t',qd1,qd2);
% q=mtraj(@tpoly,q1,q2,t';     %利用jtraj求关节轨迹、速度、加速度 五次零速到点（可优化
qd = FDMinter3(tspan',q');
qdd = FDMinter3(tspan',qd);
qd = qd';
qdd = qdd';
for i = 1:6
    disp(['max vel in joint ' num2str(i) ' is ' num2str(norm(qd(:,i),inf)/pi*180)])
    disp(['max acc in joint ' num2str(i) ' is ' num2str(norm(qdd(:,i),inf)/pi*180)])
end

%% 录制动画
% sim('my_calc_torquecontrol');
close all;
qout = q;
aviobj = VideoWriter('demo.avi');
open(aviobj);
T = zeros(4,4,size(qout,1));
wpts = zeros(3,size(qout,1));
load wpts.mat
for i = 1:size(qout,1)
    R.plot(qout(i,:));
    hold on
    plot3(wpts(1,:),wpts(2,:),wpts(3,:),'r')
    hold off
    T(:,:,i) = R.fkine(qout(i,:));
    wpts(:,i) = transl(T(:,:,i));
    view(135,45)
    F = getframe;
    writeVideo(aviobj,F);
end
close(aviobj)

%% 逆动力学
% %关节1
% [Q2,Q3] = meshgrid(R.qlim(2,1):0.1:R.qlim(2,2), R.qlim(3,1):0.1:R.qlim(3,2));
% M11 = zeros(numrows(Q2),numcols(Q3));
% for i=1:numrows(Q2)
%     for j=1:numcols(Q3)
%         M = R.inertia([0 Q2(i,j) Q3(i,j) 0 0 0]);
%         M11(i,j) = M(1,1);
%     end
% end
% figure()
% surfl(Q2, Q3, M11);
% xlabel('Q2,rad');ylabel('Q3,rad');zlabel('M11,kgm^2');
% title('load inertia on joint1')

%% 关节2
% Q3 = R.qlim(3,1):0.1:R.qlim(3,2);
% M22 = zeros(numcols(Q3));
% for j=1:numcols(Q3)
%     M = R.inertia([0 0 Q3(j) 0 0 0]);
%     M22(j) = M(2,2);
% end
% figure()
% plot(Q3, M22);
% xlabel('Q3,rad');ylabel('M22,kgm^2');
% grid on
% title('load inertia on joint2')

%% 关节3
% [Q4,Q5] = meshgrid(R.qlim(4,1):0.1:R.qlim(4,2), R.qlim(5,1):0.1:R.qlim(5,2));
% M33 = zeros(numrows(Q4),numcols(Q5));
% for i=1:numrows(Q4)
%     for j=1:numcols(Q5)
%         M = R.inertia([0 0 0 Q4(i,j) Q5(i,j) 0]);
%         M33(i,j) = sum(M(3,:));
%     end
% end
% figure()
% surfl(Q4, Q5, M33);
% xlabel('Q4,rad');ylabel('Q5,rad');zlabel('M33,kgm^2');
% title('load inertia on joint3')

%% 关节4
% [Q5,Q6] = meshgrid(R.qlim(5,1):0.1:R.qlim(5,2), R.qlim(6,1):0.1:R.qlim(6,2));
% M44 = zeros(numrows(Q5),numcols(Q6));
% for i=1:numrows(Q5)
%     for j=1:numcols(Q6)
%         M = R.inertia([0 0 0 0 Q5(i,j) Q6(i,j)]);
%         M44(i,j) = sum(M(4,:));
%     end
% end
% figure()
% surfl(Q5, Q6, M44);
% xlabel('Q5,rad');ylabel('Q6,rad');zlabel('M4,kgm^2');
% title('load inertia on joint4')

%% 关节5
% Q6 = R.qlim(6,1):0.1:R.qlim(6,2);
% M55 = zeros(numcols(Q6));
% for j=1:numcols(Q6)
%     M = R.inertia([0 0 0 0 0 Q6(j)]);
%     M55(j) = sum(M(5,:));
% end
% figure()
% plot(Q6, M55);
% xlabel('Q6,rad');ylabel('M5,kgm^2');
% grid on
% title('load inertia on joint5')

%% 关节6
% Q6 = R.qlim(6,1):0.1:R.qlim(6,2);
% M55 = zeros(numcols(Q6));
% for j=1:numcols(Q6)
%     M = R.inertia([0 0 0 0 0 Q6(j)]);
%     M55(j) = sum(M(6,:));
% end
% figure()
% plot(Q6, M55);
% xlabel('Q6,rad');ylabel('M5,kgm^2');
% grid on
% title('load inertia on joint5')

%% gravity load
% [Q2,Q3] = meshgrid(R.qlim(2,1):0.1:R.qlim(2,2), R.qlim(3,1):0.1:R.qlim(3,2));
% g2 = zeros(numrows(Q2),numcols(Q3));
% g3 = zeros(numrows(Q2),numcols(Q3));
% g4 = zeros(numrows(Q2),numcols(Q3));
% g5 = zeros(numrows(Q2),numcols(Q3));
% for i=1:numrows(Q2)
%     for j=1:numcols(Q3)
%         g = R.gravload([0 Q2(i,j) Q3(i,j) 0 0 0]);
%         g2(i,j) = g(2);
%         g3(i,j) = g(3);
%         g4(i,j) = g(4);
%         g5(i,j) = g(5);
%     end
% end
% figure()
% surfl(Q2, Q3, g2);
% xlabel('Q2,rad');ylabel('Q3,rad');zlabel('grav load,Nm');
% title('gravity load on joint2')
% figure()
% surfl(Q2, Q3, g3);
% xlabel('Q2,rad');ylabel('Q3,rad');zlabel('grav load,Nm');
% title('gravity load on joint3')
% figure()
% surfl(Q2, Q3, g4);
% xlabel('Q2,rad');ylabel('Q3,rad');zlabel('grav load,Nm');
% title('gravity load on joint4')
% figure()
% surfl(Q2, Q3, g5);
% xlabel('Q2,rad');ylabel('Q3,rad');zlabel('grav load,Nm');
% title('gravity load on joint5')

%% 外力项
% q = [0.01,0.01,0.01,0.01,0.01,0.01];
% mext = [0,0,0,22,22,10];
% tau = transpose(R.jacob0(q))*mext';

%%
% Fext = [10*9.81;0;0;0;0;0]; % 末端负载
Fext = [0;0;0;22;22;10]; % 末端负载
Grav = [0,0,9.81];
R.gravity = Grav;
R.payload(10,[0,0,0]);
tau = R.rne(q, qd, qdd, 'fext',Fext);

%% 输出数据
for i = 1:6
    ftest = fopen(['joint_param_' num2str(i) '.txt'],'w');
    fprintf(ftest,' Time           q%d             qd%d            qdd%d           torque%d\n',i,i,i,i);
    for j = 1:length(tspan)
        fprintf(ftest,'%14.7E %14.7E %14.7E %14.7E %14.7E\n',tspan(j),q(j,i),qd(j,i),qdd(j,i),tau(j,i));
    end
    fclose(ftest);
end

%% 出图
t = tspan;
figure()
subplot(2,1,1)
for i = 1:6
    plot(t,tau(:,i))
    hold on
    labels{i} = ['torque' num2str(i)];
end
legend(labels);grid on
title('inverse Dynamic torque analysis')
xlabel('time,s');ylabel('torque,N*m')

subplot(2,1,2)
power = zeros(6,length(t));
for i = 1:length(t)
    qdi = qd(i,:);
    taui = tau(i,:);
    power(:,i) = taui.*qdi;
end

for i = 1:6
    plot(t,power(i,:))
    hold on
    labels{i} = ['power' num2str(i)];
end
legend(labels);grid on
title('inverse Dynamic power analysis')
xlabel('time,s');ylabel('power,W')

rmpath(genpath('.'))