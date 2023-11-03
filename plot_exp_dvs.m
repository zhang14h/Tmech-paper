close all;
figure('position',[0 0 1000 500]);

% p1 = plot(dv1,'-r','LineWidth',3); m1 = "K_{o1} = 10.5I";
% hold on;
% p2 = plot(dv2,':g','LineWidth',3); m2 = "K_{o2} = 5.5I";
% p3 = plot(dv3,'-.b','LineWidth',3); m3 = "K_{o3} = 2.5I";
% 
% p1 = plot(dv1,'-r','LineWidth',3); m1 = "l_1 = 0.006";
% hold on;
% p2 = plot(dv2,':g','LineWidth',3); m2 = "l_2 = 0.003";
% p3 = plot(dv3,'-.b','LineWidth',3); m3 = "l_3 = 0.001";

% p1 = plot(dv1,'-r','LineWidth',3); m1 = "\eta_1 = 10";
% hold on;
% p2 = plot(dv2,':g','LineWidth',3); m2 = "\eta_2 = 6";
% p3 = plot(dv3,'-.b','LineWidth',3); m3 = "\eta_3 = 2";


p1 = plot(dv1,'-r','LineWidth',3); m1 = "Proposed Method";
hold on;
p4 = plot(dv4,'-.k','LineWidth',3); m4 = "Observer in Section 3.2";
p5 = plot(dv5,'-c','LineWidth',3); m5 = "Observer in [46]";
% 
% legend([p1,p2,p3],[m1,m2,m3],'Fontsize',25);
legend([p1,p4,p5],[m1,m4,m5],'Fontsize',26);
grid on;
% set(gca,'fontsize',25,'FontWeight', 'bold') 
set(gca,'fontsize',30) 
% 
xlabel('Time', 'Fontsize', 25,'FontAngle','italic', 'FontWeight', 'bold');
ylabel('||Eo||_F', 'Fontsize', 25, 'FontAngle','italic', 'FontWeight', 'bold');

figure('position',[0 0 1000 500]);

% sensorshape(SE_3t(:,:,1),focall,focalh,hov,vov,Rad,1);
% sensorshape(SE_3s(:,:,1),focall,focalh,hov,vov,Rad,2);

% sensorshape(SE3_t(:,:,iter),focall,focalh,hov,vov,Rad,1);

% sensorshape(SE3_s(:,:,iter),focall,focalh,hov,vov,Rad,2);
q1 = plot3(px,py,pz,'-s','LineWidth',3); t1 = "T_{ref}";
hold on;
for i = 1:length(landmark)
   scatter3(y(1,i),y(2,i),y(3,i),40,'filled','Marker','o','MarkerFaceColor',color_uncovered_landmark_face/255,'MarkerEdgeColor',color_uncovered_landmark_edge/255,'LineWidth',1);
end;
scatter3(obstacle(1,:),obstacle(2,:),obstacle(3,:),40,'filled','Marker','o','MarkerFaceColor',color_obstacle/255,'MarkerEdgeColor',color_obstacle/255,'LineWidth',1);

view (0,90);
% q2 = plot3(hpx(1,:),hpy(1,:),hpz(1,:),'-r','LineWidth',3);  t2 = "T_{K_o1}";
% q3 = plot3(hpx(2,:),hpy(2,:),hpz(2,:),':g','LineWidth',3);  t3 = "T_{K_o2}";
% q4 = plot3(hpx(3,:),hpy(3,:),hpz(3,:),'-.b','LineWidth',3); t4 = "T_{K_o3}";

q2 = plot3(hpx(1,:),hpy(1,:),hpz(1,:),'-r','LineWidth',3);  t2 = "Proposed Method";
q5 = plot3(hpx(4,:),hpy(4,:),hpz(4,:),'-.k','LineWidth',3); t5 = "Observer in Section 3.2";
q6 = plot3(hpx(5,:),hpy(5,:),hpz(5,:),'-c','LineWidth',3); t6 = "Observer in [46]";
legend([q1,q2,q5,q6],[t1,t2,t5,t6],'Fontsize',20);
grid on;
set(gca,'fontsize',30)
% legend([q1,q2,q3,q4],[t1,t2,t3,t4]);


% figure();
%  plot (robs,'-r','linewidth',1);
% figure();
%  plot (num);
