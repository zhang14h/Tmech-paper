alpha = 0.001;

figure('position',[500 100 1000 800]);

for i = 1:iter
    
    sensorshape(SE3_t(:,:,i),focall,focalh,hov,vov,Rad,1);
    xlim([0,10]);
    zlim([-5,5]);
    view (0,90);
    sensorshape(SE3_s(:,:,i),focall,focalh,hov,vov,Rad,2);
    scatter3(obstacle(1,:),obstacle(2,:),obstacle(3,:),40,'filled','Marker','o','MarkerFaceColor',color_obstacle/255,'MarkerEdgeColor',color_obstacle/255,'LineWidth',1);
    dv4(i) = norm(SE3_t(:,:,i)-SE3_s(:,:,i),'fro');
    
    
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
          omega1(:,:,i) = omega1(:,:,i)+alpha*(SE3_s(:,:,i)*([ydet_real(:,j); 1]-y_estimated(:,j)))*transpose(yind(:,j)); 
        end;
        scatter3(yind(1,:),yind(2,:),yind(3,:),40,'filled','Marker','o','MarkerFaceColor',color_covered_landmark_face/255,'MarkerEdgeColor',color_covered_landmark_edge/255,'LineWidth',1);
    end;
    robs(i) = rank(yind);
    num(i) = length(ID);
    omega(:,:,i) =   projection(1/4*((omega1(:,:,i) - omega1(:,:,i).')));
    psi = inv(SE3_s(:,:,i))*omega(:,:,i)*SE3_s(:,:,i);
    
    SE3_s(:,:,i+1) = SE3_s(:,:,i)*expm(U_real(:,:,i) - psi ); 
    
    hold off;
    pause(0.1);
end;


for i = 1:iter
    px(i) = SE3_t(1,4,i);
    py(i) = SE3_t(2,4,i);
    pz(i) = SE3_t(3,4,i);
    hpx(4,i) = SE3_s(1,4,i);
    hpy(4,i) = SE3_s(2,4,i);
    hpz(4,i) = SE3_s(3,4,i);
end;