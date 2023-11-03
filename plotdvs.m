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


% p1 = plot(dv1,'-r','LineWidth',3); m1 = "Proposed Method";
p2 = plot(dv2,':g','LineWidth',3); m2 = "K_o = 15.5I";
hold on;
p4 = plot(dv4,'-.k','LineWidth',3); m4 = "Observer in [46]";
p5 = plot(dv5,'-r','LineWidth',3); m5 = "K_o = 6.5I";
% 
% legend([p1,p2,p3],[m1,m2,m3],'Fontsize',26);
legend([p4,p5,p2],[m4,m5,m2],'Fontsize',25);
grid on;
% set(gca,'fontsize',30,'FontWeight', 'bold') 
set(gca,'fontsize',30) 
% 
xlabel('Time', 'Fontsize', 25,'FontAngle','italic', 'FontWeight', 'bold');
ylabel('||Eo||_F', 'Fontsize', 25, 'FontAngle','italic', 'FontWeight', 'bold');


figure('position',[0 0 1000 800]);


% sensorshape(SE_3t(:,:,1),focall,focalh,hov,vov,Rad,1);
% sensorshape(SE_3s(:,:,1),focall,focalh,hov,vov,Rad,2);

sensorshape(SE_3t(:,:,iter),focall,focalh,hov,vov,Rad,1);
view(0,90);
sensorshape(SE_3s(:,:,iter),focall,focalh,hov,vov,Rad,2);
grid on;
hold on;
for i = 1:length(landmark)
   scatter3(y(1,i),y(2,i),y(3,i),40,'filled','Marker','o','MarkerFaceColor',color_uncovered_landmark_face/255,'MarkerEdgeColor',color_uncovered_landmark_edge/255,'LineWidth',1);
end;
for k = 1:lengthavoid
          if p_obstacle(1,k) >= focall & p_obstacle(1,k) <= focalh & abs(p_obstacle(2,k)) <= p_obstacle(1,k)*tan(hov/2) & abs(p_obstacle(3,k)) <= p_obstacle(1,k)*tan(vov/2);
             scatter3(y_obstacle(1,k),y_obstacle(2,k),y_obstacle(3,k),40,'filled','Marker','o','MarkerFaceColor',color_covered_obstacle/255,'MarkerEdgeColor',color_obstacle/255,'LineWidth',1); 
          else    
             scatter3(y_obstacle(1,k),y_obstacle(2,k),y_obstacle(3,k),40,'filled','Marker','o','MarkerFaceColor',color_obstacle/255,'MarkerEdgeColor',color_obstacle/255,'LineWidth',1); 
          end;   
end;

q1 = plot3(px,py,pz,'-k','LineWidth',3); t1 = "T_{ref}";


% q2 = plot3(hpx(1,:),hpy(1,:),hpz(1,:),'-r','LineWidth',3);  t2 = "T_{l_1}";
% q3 = plot3(hpx(2,:),hpy(2,:),hpz(2,:),':g','LineWidth',3);  t3 = "T_{l_2}";
% q4 = plot3(hpx(3,:),hpy(3,:),hpz(3,:),'-.b','LineWidth',3); t4 = "T_{l_3}";
legend([q1,q2,q3,q4],[t1,t2,t3,t4]);


figure();
 plot (robs,'-r','linewidth',1);
figure();
 plot (num);
