%%%% Run initialization first

%%
%%%% observer design parameters
Q = 0.01 * [1 4 3 1; 4 3 1 1; 3 1 2 1; 1 1 1 3]; 
Q=  Q+1*eye(4);  %K_o in the paper%
gammau = 4;      %eta in the paper%
%%
%%%% O_C^B calculatiion

%%
%%
%%%% varrho in the obstacle avoidance algorithm

%%
figure()

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
        dv2(i) = norm(SE_3t(:,:,i)-SE_3s(:,:,i),'fro');
        omega1(:,:,i)=zeros(4,4);
        omega2(:,:,i)=zeros(4,4);
        
        
%         sensorshape(SE_3t(:,:,i),focall,focalh,hov,vov,Rad,1);
       
        counter = 0;
        p = 0;
%%%%    plotting X        
        
%%        
%%%%%%%%%%%%%%%%  iterations with Error Detection      
%       obs3(:,i) = 0;
        U_avoid(:,:,i) = zeros(4,4);
        if i >= 400 && i<= 700    
          for j = 1:landmarks
           cframept(:,j) =  inv(SE_3t(:,:,i)) *  y(:,j);
           cframeps(:,j) =  inv(SE_3s(:,:,i)) *  y(:,j);
           deltay(:,j,i) =  snr*[0.3*cos(i*j/2) 0.2*sin(i*j/5) 0.1*sin(i*j/4) 0].';
%%       
%%%%%   Error Detection landmarks
%             cframept(:,10) = [4;1.5;-1;1];
%             cframept(:,150) = [3.5;-2;1;1];
%             cframept(:,100) = [2;-2;1.5;1];
%             cframept(:,91) = [3;-1.5;-1;1];
%              cframept(:,11) = [2.5;-2;-1;1];
%             cframept(:,67) = [2.8;-2;-1.8;1];
%%
            if cframept(1,j) >= focall & cframept(1,j) <= focalh & abs(cframept(2,j)) <= cframept(1,j)*tan(hov/2) & abs(cframept(3,j)) <= cframept(1,j)*tan(vov/2);     %%Identified landmarks
                
%                
                if cframeps(1,j) >= focall & cframeps(1,j) <= focalh & abs(cframeps(2,j)) <= cframeps(1,j)*tan(hov/2) & abs(cframeps(3,j)) <= cframeps(1,j)*tan(vov/2);   %%Landmarks estimated to be identifed in the observer
%                     plot3(y(1,j),y(2,j),y(3,j),'o','Color','r','MarkerSize',5);
                    omega1(:,:,i) = omega1(:,:,i)+alpha*(SE_3s(:,:,i)*(cframept(:,j)+deltay(:,j,i))-y(:,j))*transpose(y(:,j));                                       %% Psi in the paper
                    counter = counter+1;
                    p(counter) = j;
%                      
                else 
%                     plot3(y(1,j),y(2,j),y(3,j),'o','Color','b','MarkerSize',5); %avoid points missing display

                    diss =  sqrt((cframeps(1,j) - focalm(1,:))^2 + (cframeps(2,j))^2 + (cframeps(3,j))^2);
%                     
                    omega2(:,:,i) = omega2(:,:,i)+alpha*exp((gammau-diss)/gammau)*(SE_3s(:,:,i)*(cframept(:,j)+deltay(:,j,i))-y(:,j))*transpose(y(:,j));    %%Q(\haty, y) in the paper
%                     
                    counter = counter+1;
%                    
                end;
                yprime(:,counter,i) = [y(:,j)];
                robs(i) = rank(yprime(:,:,i));
             else 
%                     plot3(y(1,j),y(2,j),y(3,j),'o','Color','k','MarkerSize',5);
%                     
                    
             end;
                            
           end;
%%   
%%%%%%%%   Obstacle Avoidance Algorithms
           for k = 1:lengthavoid
               p_obstacle(:,k) = inv(SE_3t(:,:,i))*y_obstacle(:,k)+snr*[0.3*cos(i*j/2) 0.2*sin(i*j/5) 0*sin(i*j/4) 0].';
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
            cframeps(:,j) =  inv(SE_3s(:,:,i)) *  y(:,j);
            cframept(:,j) =  inv(SE_3t(:,:,i)) *  y(:,j);
            deltay(:,j,i) = snr*[0.3*cos(i*j/2) 0.2*sin(i*j/5) 0.1*sin(i*j/4) 0].';
            
            
             if cframept(1,j) >= focall & cframept(1,j) <= focalh & abs(cframept(2,j)) <= cframept(1,j)*tan(hov/2) & abs(cframept(3,j)) <= cframept(1,j)*tan(vov/2);
                
%                 
                if cframeps(1,j) >= focall & cframeps(1,j) <= focalh & abs(cframeps(2,j)) <= cframeps(1,j)*tan(hov/2) & abs(cframeps(3,j)) <= cframeps(1,j)*tan(vov/2);
%                     plot3(y(1,j),y(2,j),y(3,j),'o','Color','r','MarkerSize',5);
                    omega1(:,:,i) = omega1(:,:,i)+alpha*(SE_3s(:,:,i)*(cframept(:,j)+deltay(:,j,i))-y(:,j))*transpose(y(:,j));
                    counter = counter+1;
                    p(counter) = j;
%                     
                else 
%                     plot3(y(1,j),y(2,j),y(3,j),'o','Color','b','MarkerSize',5); %avoid points missing display
%                                    
%                     diss =  sqrt((cframeps(1,j) - focalm(1,:))^2/((focalh-focall)/2)^2 + (cframeps(2,j))^2/(focalm(1,:)*tan(hov/2))^2 + (cframeps(3,j))^2/(focalm(1,:)*tan(vov/2))^2);
                   diss =  sqrt((cframeps(1,j) - focalm(1,:))^2 + (cframeps(2,j))^2 + (cframeps(3,j))^2);                
                    omega2(:,:,i) = omega2(:,:,i)+alpha*exp((gammau-diss)/gammau)*(SE_3s(:,:,i)*(cframept(:,j)+deltay(:,j,i))-y(:,j))*transpose(y(:,j));
%                     omega2(:,:,i) = omega2(:,:,i)+alpha*exp(0.8-(diss-dist))*(cframept(:,j))*transpose(y(:,j));
                    counter = counter+1;
%                    
                end;
                yprime(:,counter,i) = [y(:,j)];
                robs(i) = rank(yprime(:,:,i));
             else 
%                     plot3(y(1,j),y(2,j),y(3,j),'o','Color','k','MarkerSize',5);
%                    
%                     
             end;
            
%           obs3(:,i) = obs3(:,i) + dis(j);
          end;
          for k = 1:lengthavoid
               p_obstacle(:,k) = inv(SE_3t(:,:,i))*y_obstacle(:,k)+snr*[0.3*cos(i*j/2) 0.2*sin(i*j/5) 0.0*sin(i*j/4) 0].';  
               if p_obstacle(1,k) >= focall & p_obstacle(1,k) <= focalh & abs(p_obstacle(2,k)) <= p_obstacle(1,k)*tan(hov/2) & abs(p_obstacle(3,k)) <= p_obstacle(1,k)*tan(vov/2);
%                   omega_avoid(:,:,k) =  p_obstacle(:,k)*(p_obstacle(:,k)-[0;0;0;1]).';   
%                   a_avoid = exp(piu - 1*norm(p_obstacle(:,k)-[0;0;0;1],'fro'));
                  omega_avoid(:,:,k) =  p_obstacle(:,k)*(p_obstacle(:,k)-focalm).';
                  a_avoid = exp(piu - 1*norm(p_obstacle(:,k)-focalm,'fro'));      
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
        se_3s(:,:,i) =  U(:,:,i)-inv(SE_3s(:,:,i))*omega(:,:,i)*SE_3s(:,:,i)-inv(SE_3s(:,:,i))*projection (1*[1 0 0 0;0 1 0 0;0 0 1 0; 0 0 0 1]*1/4*((Q*omega2(:,:,i) - omega2(:,:,i).'*Q.')))*SE_3s(:,:,i); %U-Psi-Q(haty,y)
              
        SE_3s(:,:,i+1) = SE_3s(:,:,i)*expm(se_3s(:,:,i));
%         sensorshape(SE_3s(:,:,i),focall,focalh,hov,vov,Rad,2);  
%%   
%%%%%%%%%%    System Update
%         if i >= 3000 && i<5500
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
%              plot3(y_obstacle(1,k),y_obstacle(2,k),y_obstacle(3,k),'o','Color','r','MarkerSize',10);
%           else    
%              plot3(y_obstacle(1,k),y_obstacle(2,k),y_obstacle(3,k),'o','Color','k','MarkerSize',10); 
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

L2_3 = sqrt(sum(dv2*dv2.'));
Linf_3 = max(dv2);
display (L2_3);
display (Linf_3);

figure();
% % plot (dv1,'-r','linewidth',1);
% % hold on;
plot (dv2,':k','linewidth',1);
% figure();
% plot (robs,'-r','linewidth',1);
figure();
for i = 1:iter
    px(i) = SE_3t(1,4,i);
    py(i) = SE_3t(2,4,i);
    pz(i) = SE_3t(3,4,i);
    hpx(i) = SE_3s(1,4,i);
    hpy(i) = SE_3s(2,4,i);
    hpz(i) = SE_3s(3,4,i);
end;
%  view(0,90)
 plot3(px,py,pz);
 hold on
 for j = 1:lengthavoid
      plot3(y_obstacle(1,j),y_obstacle(2,j),y_obstacle(3,j),'o','Color','r','MarkerSize',10);
 end; 
%  sensorshape(SE_3t(:,:,1),focall,focalh,hov,vov,Rad,1);
% %  sensorshape(SE_3t(:,:,100),focall,focalh,hov,vov,Rad,1);
% %  sensorshape(SE_3t(:,:,500),focall,focalh,hov,vov,Rad,1);
%  sensorshape(SE_3t(:,:,1000),focall,focalh,hov,vov,Rad,1);
% %  sensorshape(SE_3t(:,:,2000),focall,focalh,hov,vov,Rad,1);
%  sensorshape(SE_3t(:,:,3000),focall,focalh,hov,vov,Rad,1);
 plot3(hpx,hpy,hpz,'-.k');
%  hold on;
%  plot3(hpx,hpy,hpz);
%  xlim([-3,3]);
%  ylim([-3,3]);
%  zlim([-3,3])
 figure
 for j = 1:landmarks
            
                    plot3(y(1,j),y(2,j),y(3,j),'o','Color','k','MarkerSize',5);
                    hold on;
%                    
end;
figure();
 plot (robs,'-r','linewidth',1);
figure();
 plot (num);
