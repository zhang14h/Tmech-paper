%%%% Run initialization first

%%
%%%% observer design parameters

%%
% figure('position',[0 0 1000 800]);
alpha = 0.001;
for i = 1:iter
%%       
        TRt(:,:,i) = [SE_3t(1,1,i) SE_3t(1,2,i) SE_3t(1,3,i);
                 SE_3t(2,1,i) SE_3t(2,2,i) SE_3t(2,3,i);
                 SE_3t(3,1,i) SE_3t(3,2,i) SE_3t(3,3,i)];
             
        TPt(:,i) = [SE_3t(1,4,i) SE_3t(2,4,i) SE_3t(3,4,i)];
        
        TRs(:,:,i) = [SE_3s(1,1,i) SE_3s(1,2,i) SE_3s(1,3,i);
                 SE_3s(2,1,i) SE_3s(2,2,i) SE_3s(2,3,i);
                 SE_3s(3,1,i) SE_3s(3,2,i) SE_3s(3,3,i)];
             
        TPs(:,i) = [SE_3s(1,4,i) SE_3s(2,4,i) SE_3s(3,4,i)];  
%%%%    error calculation          
        error4(:,:,i) =  SE_3s(:,:,i)*inv(SE_3t(:,:,i));
%%        
%         vd3(i) = abs(visiondistance(TRt(:,:,i),TRs(:,:,i),TPt(:,i),TPs(:,i)));
        dv4(i) = norm(SE_3t(:,:,i)-SE_3s(:,:,i),'fro');
        omega1(:,:,i)=zeros(4,4);
        omega2(:,:,i)=zeros(4,4);
        
        
        
%         sensorshape(SE_3t(:,:,i),focall,focalh,hov,vov,Rad,1);
%         view(0,90);
%         grid;
        
        counter = 0;
        p = 0;
%%%%    plotting X        
        
%%        
%%%%%%%%%%%%%%%%  iterations with Error Detection      
%       obs3(:,i) = 0;
        U_avoid(:,:,i) = zeros(4,4);
        if (i >= 100 && i<= 110) || (i >= 210 && i<= 230) || (i >= 270 && i<= 300)|| (i >= 430 && i<= 440)
          for j = 1:54
           cframept(:,j) =  inv(SE_3t(:,:,i)) *  [y(1,j) y(2,j) y(3,j) 1].';
           cframeps(:,j) =  inv(SE_3s(:,:,i)) *  [y(1,j) y(2,j) y(3,j) 1].';
           deltay(:,j,i) =  snr*[0.3*cos(i*j/2) 0.2*sin(i*j/5) 0.1*sin(i*j/4) 0].';
%%       
%%%%   Error Detection landmarks
%             cframept(:,55) = [2.5;1;-1;1];
%             cframept(:,56) = [3.1;1;1;1];
%             cframept(:,57) = [3.7;0.5;0.5;1];
%             cframept(:,58) = [2.5;1.2;-1;1];
%              cframept(:,59) = [3;1;0.5;1];

%%
%             if cframept(1,j) >= focall & cframept(1,j) <= focalh & abs(cframept(2,j)) <= cframept(1,j)*tan(hov/2) & abs(cframept(3,j)) <= cframept(1,j)*tan(vov/2);
%                     
%             if cframept(1,j)^2 + cframept(2,j)^2 + cframept(3,j)^2 <= Rad^2; 
%                       scatter3(y(1,j),y(2,j),y(3,j),40,'filled','Marker','o','MarkerFaceColor',color_covered_landmark_face/255,'MarkerEdgeColor',color_covered_landmark_edge/255,'LineWidth',1);
                      omega1(:,:,i) = omega1(:,:,i)+alpha*(SE_3s(:,:,i)*(cframept(:,j)+deltay(:,j,i))-[y(1,j) y(2,j) y(3,j) 1].')*transpose([y(1,j) y(2,j) y(3,j) 1].');    
%                      
                      counter = counter+1;
                      yprime(:,counter,i) = [[y(1,j) y(2,j) y(3,j) 1].'];
                      
                      robs(i) = rank(yprime(:,:,i));
%             else 
%                       scatter3(y(1,j),y(2,j),y(3,j),40,'filled','Marker','o','MarkerFaceColor',color_uncovered_landmark_face/255,'MarkerEdgeColor',color_uncovered_landmark_edge/255,'LineWidth',1);
                     
%                     
%             end;
                            
           end;
%%   
%%%%%%%%   Obstacle Avoidance Algorithms
           for k = 1:lengthavoid
               p_obstacle(:,k) = inv(SE_3t(:,:,i))*y_obstacle(:,k)+0*[0.3*cos(i*j/2) 0.2*sin(i*j/5) 0*sin(i*j/4) 0].';  
               if p_obstacle(1,k) >= focall & p_obstacle(1,k) <= focalh & abs(p_obstacle(2,k)) <= p_obstacle(1,k)*tan(hov/2) & abs(p_obstacle(3,k)) <= p_obstacle(1,k)*tan(vov/2);
%                  omega_avoid(:,:,k) =  p_obstacle(:,k)*(p_obstacle(:,k)-[0;0;0;1]).';   
%                  a_avoid = exp(piu - 1*norm(p_obstacle(:,k)-[0;0;0;1],'fro'));
                 omega_avoid(:,:,k) =  p_obstacle(:,k)*(p_obstacle(:,k)-focalm).';
                 a_avoid = exp(piu - 1*norm(p_obstacle(:,k)-focalm,'fro'));     
                 U_avoid(:,:,i) = U_avoid(:,:,i) + a_avoid*projection(1/4*((omega_avoid(:,:,k) - omega_avoid(:,:,k).')));
               end;  
           end;
           
        else
%%  
%%%%%%%%%%%%%  Iterations without error detection
          for j = 1:landmarks
            cframeps(:,j) =  inv(SE_3s(:,:,i)) *  [y(1,j) y(2,j) y(3,j) 1].';
            cframept(:,j) =  inv(SE_3t(:,:,i)) *  [y(1,j) y(2,j) y(3,j) 1].';
            deltay(:,j,i) = snr*[0.3*cos(i*j/2) 0.2*sin(i*j/5) 0.1*sin(i*j/4) 0].';
            
            
%              if cframept(1,j) >= focall & cframept(1,j) <= focalh & abs(cframept(2,j)) <= cframept(1,j)*tan(hov/2) & abs(cframept(3,j)) <= cframept(1,j)*tan(vov/2);
%                     
%             if cframept(1,j)^2 + cframept(2,j)^2 + cframept(3,j)^2 <= Rad^2; 
%                       scatter3(y(1,j),y(2,j),y(3,j),40,'filled','Marker','o','MarkerFaceColor',color_covered_landmark_face/255,'MarkerEdgeColor',color_covered_landmark_edge/255,'LineWidth',1);
                      omega1(:,:,i) = omega1(:,:,i)+alpha*(SE_3s(:,:,i)*(cframept(:,j)+deltay(:,j,i))-[y(1,j) y(2,j) y(3,j) 1].')*transpose([y(1,j) y(2,j) y(3,j) 1].');    
%                      
                      counter = counter+1;
                      yprime(:,counter,i) = [[y(1,j) y(2,j) y(3,j) 1].'];
                      
                      robs(i) = rank(yprime(:,:,i));
%              else 
%                       scatter3(y(1,j),y(2,j),y(3,j),40,'filled','Marker','o','MarkerFaceColor',color_uncovered_landmark_face/255,'MarkerEdgeColor',color_uncovered_landmark_edge/255,'LineWidth',1);
                     
%                     
%             end;
            
%           obs3(:,i) = obs3(:,i) + dis(j);
          end;
          for k = 1:lengthavoid
               p_obstacle(:,k) = inv(SE_3t(:,:,i))*y_obstacle(:,k)+0*[0.3*cos(i*j/2) 0.2*sin(i*j/5) 0*sin(i*j/4) 0].'; 
               if p_obstacle(1,k) >= focall & p_obstacle(1,k) <= focalh & abs(p_obstacle(2,k)) <= p_obstacle(1,k)*tan(hov/2) & abs(p_obstacle(3,k)) <= p_obstacle(1,k)*tan(vov/2);
                  omega_avoid(:,:,k) =  p_obstacle(:,k)*(p_obstacle(:,k)-[1;0;0;1]).';   
                  a_avoid = exp(piu - 1*norm(p_obstacle(:,k)-[1;0;0;1],'fro'));
%                   omega_avoid(:,:,k) =  p_obstacle(:,k)*(p_obstacle(:,k)-focalm).';
%                   a_avoid = exp(piu - 1*norm(p_obstacle(:,k)-focalm,'fro'));      
                 U_avoid(:,:,i) = U_avoid(:,:,i) + a_avoid*projection(1/4*((omega_avoid(:,:,k) - omega_avoid(:,:,k).')));
               end;  
          end;
        end;
        num(i) = counter; 
        
        omega(:,:,i) =   projection(1/4*((omega1(:,:,i) - omega1(:,:,i).')));
%%         
%%%%%%%%%    Observer update       
        U_avoid(2,4,i) = 0;
        U_avoid(3,4,i) = 0;

        U(:,:,i) = [w_hat_t(:,:,i) v_t(:,1,i);
                  [0,0,0]        0   ] + p_avoid*U_avoid(:,:,i);       
        se_3s(:,:,i) =  U(:,:,i)-inv(SE_3s(:,:,i))*omega(:,:,i)*SE_3s(:,:,i); %U-Psi
              
        SE_3s(:,:,i+1) = SE_3s(:,:,i)*expm(se_3s(:,:,i));
%         sensorshape(SE_3s(:,:,i),focall,focalh,hov,vov,Rad,2);  
%%   
%%%%%%%%%%    System Update
%         if i >= 100 && i<350
%             se3_t(:,:,i) = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];
%         else    
%             se3_t(:,:,i) = U(:,:,i)+[delta_hat_t(:,:,i) v_delta(:,1,i);
%                   [0,0,0]        0   ];
%         end;
        se3_t(:,:,i) = U(:,:,i)+[delta_hat_t(:,:,i) v_delta(:,1,i);
                  [0,0,0]        0   ];
        SE_3t(:,:,i+1) = SE_3t(:,:,i)*expm(se3_t(:,:,i));
             
%       view(0,90)  
%       xlim([-20,40]);
%       ylim([-20,40]);  
%       for k = 1:lengthavoid
%           if p_obstacle(1,k) >= focall & p_obstacle(1,k) <= focalh & abs(p_obstacle(2,k)) <= p_obstacle(1,k)*tan(hov/2) & abs(p_obstacle(3,k)) <= p_obstacle(1,k)*tan(vov/2);
%              scatter3(y_obstacle(1,k),y_obstacle(2,k),y_obstacle(3,k),40,'filled','Marker','o','MarkerFaceColor',color_covered_obstacle/255,'MarkerEdgeColor',color_obstacle/255,'LineWidth',1); 
%           else    
%              scatter3(y_obstacle(1,k),y_obstacle(2,k),y_obstacle(3,k),40,'filled','Marker','o','MarkerFaceColor',color_obstacle/255,'MarkerEdgeColor',color_obstacle/255,'LineWidth',1); 
%           end;   
%       end;
%       hold off;
%       pause(0.001)
       
        
end;
% figure('position',[0 0 1000 800]);
% sensorshape(SE_3t(:,:,i),focall,focalh,hov,vov,Rad,1);
% hold on;
% for j = 1:landmarks
% if cframept(1,j) >= focall & cframept(1,j) <= focalh & abs(cframept(2,j)) <= cframept(1,j)*tan(hov/2) & abs(cframept(3,j)) <= cframept(1,j)*tan(vov/2);
%                 
% %                 
%                 if cframeps(1,j) >= focall & cframeps(1,j) <= focalh & abs(cframeps(2,j)) <= cframeps(1,j)*tan(hov/2) & abs(cframeps(3,j)) <= cframeps(1,j)*tan(vov/2);
%                     plot3(y(1,j),y(2,j),y(3,j),'o','Color','r','MarkerSize',5);
%                    
%                 else 
%                     plot3(y(1,j),y(2,j),y(3,j),'o','Color','b','MarkerSize',5); %avoid points missing display
% %                                    
%       
%                 end;
%                 
% end;
% end;
% sensorshape(SE_3s(:,:,i),focall,focalh,hov,vov,Rad,2);

L2_4 = sqrt(sum(dv4*dv4.')/iter);
Linf_4 = max(dv4);
display (L2_4);
display (Linf_4);

figure();
% % plot (dv1,'-r','linewidth',1);
% % hold on;
plot (dv4,':k','linewidth',1);
% figure();
% plot (robs,'-r','linewidth',1);
figure();
for i = 1:iter
    px(i) = SE_3t(1,4,i);
    py(i) = SE_3t(2,4,i);
    pz(i) = SE_3t(3,4,i);
    hpx(1,i) = SE_3s(1,4,i);
    hpy(1,i) = SE_3s(2,4,i);
    hpz(1,i) = SE_3s(3,4,i);
end;
 view(0,90)
 plot3(px,py,pz);
 hold on
 for k = 1:lengthavoid
          if p_obstacle(1,k) >= focall & p_obstacle(1,k) <= focalh & abs(p_obstacle(2,k)) <= p_obstacle(1,k)*tan(hov/2) & abs(p_obstacle(3,k)) <= p_obstacle(1,k)*tan(vov/2);
             scatter3(y_obstacle(1,k),y_obstacle(2,k),y_obstacle(3,k),40,'filled','Marker','o','MarkerFaceColor',color_covered_obstacle/255,'MarkerEdgeColor',color_obstacle/255,'LineWidth',1); 
          else    
             scatter3(y_obstacle(1,k),y_obstacle(2,k),y_obstacle(3,k),40,'filled','Marker','o','MarkerFaceColor',color_obstacle/255,'MarkerEdgeColor',color_obstacle/255,'LineWidth',1); 
          end;   
      end;
 
%  sensorshape(SE_3t(:,:,1),focall,focalh,hov,vov,Rad,1);
% %  sensorshape(SE_3t(:,:,100),focall,focalh,hov,vov,Rad,1);
% %  sensorshape(SE_3t(:,:,500),focall,focalh,hov,vov,Rad,1);
%  sensorshape(SE_3t(:,:,1000),focall,focalh,hov,vov,Rad,1);
% %  sensorshape(SE_3t(:,:,2000),focall,focalh,hov,vov,Rad,1);
%  sensorshape(SE_3t(:,:,3000),focall,focalh,hov,vov,Rad,1);
 plot3(hpx(1,:),hpy(1,:),hpz(1,:),'-.k');
%  hold on;
%  plot3(hpx,hpy,hpz);
%  xlim([-3,3]);
%  ylim([-3,3]);
%  zlim([-3,3])

