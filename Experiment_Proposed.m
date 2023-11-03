alpha = 0.001;
Q = 0.01 * [1 4 3 1; 4 3 1 1; 3 1 2 1; 1 1 1 3]; 
Q=  Q+2.5*eye(4);
gammau = 2;
figure('position',[500 100 1000 800]);
for i = 1:iter
    
    sensorshape(SE3_t(:,:,i),focall,focalh,hov,vov,Rad,1);
    xlim([0,10]);
    zlim([-5,5]);
    view (0,90);
    sensorshape(SE3_s(:,:,i),focall,focalh,hov,vov,Rad,2);
    scatter3(obstacle(1,:),obstacle(2,:),obstacle(3,:),40,'filled','Marker','o','MarkerFaceColor',color_obstacle/255,'MarkerEdgeColor',color_obstacle/255,'LineWidth',1);
    dv1(i) = norm(SE3_t(:,:,i)-SE3_s(:,:,i),'fro');
    
    
    omega1(:,:,i)=zeros(4,4);
    omega2(:,:,i)=zeros(4,4);
    
    
%     ID = [];
    ID = [landmark_camera_frame_id{:,i}];
    ydet = [landmark_camera_frame{1,i}];
    ydet_real = [ydet(3,:) ;-ydet(1,:) ;-ydet(2,:)];
    
    scatter3(y(1,:),y(2,:),y(3,:),40,'filled','Marker','o','MarkerFaceColor',color_uncovered_landmark_face/255,'MarkerEdgeColor',color_uncovered_landmark_edge/255,'LineWidth',1);
    yind = [];
    y_estimated = [];
    if length(ID) == 0;
        
    else
        for j = 1: length(ID)  
            ind = find(y(5,:)==ID(j));
            yind(:,j) = [y(1,ind) y(2,ind) y(3,ind) 1].';
            y_estimated(:,j) = inv(SE3_s(:,:,i))*yind(:,j);
            if y_estimated(1,j) >= focall & y_estimated(1,j) <= focalh & abs(y_estimated(2,j)) <= y_estimated(1,j)*tan(hov/2) & abs(y_estimated(3,j)) <= y_estimated(1,j)*tan(vov/2);
                omega1(:,:,i) = omega1(:,:,i)+alpha*(SE3_s(:,:,i)*([ydet_real(:,j); 1]-y_estimated(:,j)))*transpose(yind(:,j)); 
                scatter3(yind(1,j),yind(2,j),yind(3,j),40,'filled','Marker','o','MarkerFaceColor',color_estimated_covered_landmark_face/255,'MarkerEdgeColor',color_covered_landmark_edge/255,'LineWidth',1);
            else
                diss =  sqrt((y_estimated(1,j) - focalm(1,:))^2 + (y_estimated(2,j))^2 + (y_estimated(3,j))^2);
%                     
                omega2(:,:,i) = omega2(:,:,i)+alpha*exp((gammau-diss)/gammau)*(SE3_s(:,:,i)*([ydet_real(:,j); 1]-y_estimated(:,j)))*transpose(yind(:,j));     %%Q(\haty, y) in the paper
                scatter3(yind(1,j),yind(2,j),yind(3,j),40,'filled','Marker','o','MarkerFaceColor',color_covered_landmark_face/255,'MarkerEdgeColor',color_covered_landmark_edge/255,'LineWidth',1);      
            end;    
        end;
        
    end;
    omega(:,:,i) =   projection(1/4*((omega1(:,:,i) - omega1(:,:,i).')));
    psi = inv(SE3_s(:,:,i))*omega(:,:,i)*SE3_s(:,:,i);
    Q_y = inv(SE3_s(:,:,i))*projection (1/4*((Q*omega2(:,:,i) - omega2(:,:,i).'*Q.')))*SE3_s(:,:,i);
    
    SE3_s(:,:,i+1) = SE3_s(:,:,i)*expm(U_real(:,:,i) - psi - Q_y) ; %U-Psi-Q(haty,y)
              
    
     
    hold off;
    pause(0.1);
end;

L2_1 = sqrt(sum(dv1*dv1.')/iter);
Linf_1 = max(dv1);
display (L2_1);
display (Linf_1);

for i = 1:iter
    px(i) = SE3_t(1,4,i);
    py(i) = SE3_t(2,4,i);
    pz(i) = SE3_t(3,4,i);
    hpx(1,i) = SE3_s(1,4,i);
    hpy(1,i) = SE3_s(2,4,i);
    hpz(1,i) = SE3_s(3,4,i);
end;