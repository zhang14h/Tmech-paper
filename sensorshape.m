function  sensorgeometry = theshape(SE_3,focall,focalh,hov,vov,R,f)
         
        beta = 1;
        focallm = SE_3*[(focall+focalh)/2 ;0 ; 0 ; 1];
        focall1 = SE_3*[focall ;focall*tan(hov/2) ; focall*tan(vov/2) ; 1]; 
        focall2 = SE_3*[focall ;focall*tan(hov/2) ; -focall*tan(vov/2) ; 1];
        focall3 = SE_3*[focall ;-focall*tan(hov/2) ; -focall*tan(vov/2) ; 1];
        focall4 = SE_3*[focall ;-focall*tan(hov/2) ; focall*tan(vov/2) ; 1];
        focalh1 = SE_3*[focalh ;focalh*tan(hov/2) ; focalh*tan(vov/2) ; 1]; 
        focalh2 = SE_3*[focalh ;focalh*tan(hov/2) ; -focalh*tan(vov/2) ; 1];
        focalh3 = SE_3*[focalh ;-focalh*tan(hov/2) ; -focalh*tan(vov/2) ; 1];
        focalh4 = SE_3*[focalh ;-focalh*tan(hov/2) ; focalh*tan(vov/2) ; 1];

        sensor = [focall1(1,:) focall1(2,:) focall1(3,:); focall2(1,:) focall2(2,:) focall2(3,:); focall3(1,:) focall3(2,:) focall3(3,:); 
            focall4(1,:) focall4(2,:) focall4(3,:);focall1(1,:) focall1(2,:) focall1(3,:); focalh1(1,:) focalh1(2,:) focalh1(3,:); 
            focalh2(1,:) focalh2(2,:) focalh2(3,:);focalh3(1,:) focalh3(2,:) focalh3(3,:); focalh4(1,:) focalh4(2,:) focalh4(3,:);
            focalh1(1,:) focalh1(2,:) focalh1(3,:);focalh2(1,:) focalh2(2,:) focalh2(3,:);focall2(1,:) focall2(2,:) focall2(3,:);
            focall3(1,:) focall3(2,:) focall3(3,:);focalh3(1,:) focalh3(2,:) focalh3(3,:);focalh4(1,:) focalh4(2,:) focalh4(3,:);focall4(1,:) focall4(2,:) focall4(3,:) ];
        if f == 1;
            plot3(sensor(:,1),sensor(:,2),sensor(:,3),'r');
            hold on;
        % plot3(SE_3(1,:),SE_3(2,:),SE_3(3,:),'+','Color','k','MarkerSize',5);
        %end of sensor geometry
            plot3([SE_3(1,4),SE_3(1,4)+beta*SE_3(1,1)],[SE_3(2,4),SE_3(2,4)+beta*SE_3(2,1)],[SE_3(3,4),SE_3(3,4)+beta*SE_3(3,1)],'--r');

            plot3([SE_3(1,4),SE_3(1,4)+beta*SE_3(1,2)],[SE_3(2,4),SE_3(2,4)+beta*SE_3(2,2)],[SE_3(3,4),SE_3(3,4)+beta*SE_3(3,2)],'--g');
            plot3([SE_3(1,4),SE_3(1,4)+beta*SE_3(1,3)],[SE_3(2,4),SE_3(2,4)+beta*SE_3(2,3)],[SE_3(3,4),SE_3(3,4)+beta*SE_3(3,3)],'--b');
        end;    
        if f == 2;
            plot3(sensor(:,1),sensor(:,2),sensor(:,3),'--','Color','b','MarkerSize',3);
            hold on;
        % plot3(SE_3(1,:),SE_3(2,:),SE_3(3,:),'+','Color','k','MarkerSize',5);
        %end of sensor geometry
            plot3([SE_3(1,4),SE_3(1,4)+beta*SE_3(1,1)],[SE_3(2,4),SE_3(2,4)+beta*SE_3(2,1)],[SE_3(3,4),SE_3(3,4)+beta*SE_3(3,1)],'-','color',[10 10 10]/255);

            plot3([SE_3(1,4),SE_3(1,4)+beta*SE_3(1,2)],[SE_3(2,4),SE_3(2,4)+beta*SE_3(2,2)],[SE_3(3,4),SE_3(3,4)+beta*SE_3(3,2)],'-','color',[100 100 100]/255);
            plot3([SE_3(1,4),SE_3(1,4)+beta*SE_3(1,3)],[SE_3(2,4),SE_3(2,4)+beta*SE_3(2,3)],[SE_3(3,4),SE_3(3,4)+beta*SE_3(3,3)],'-','color',[200 200 200]/255);
        end;
        if f == 3;
           [x,y,z] = sphere(20);
           plot3(SE_3(1,4)*ones(21)+R*x,SE_3(2,4)*ones(21)+R*y,SE_3(3,4)*ones(21)+R*z,'r');  
           hold on;
           
           plot3([SE_3(1,4),SE_3(1,4)+beta*SE_3(1,1)],[SE_3(2,4),SE_3(2,4)+beta*SE_3(2,1)],[SE_3(3,4),SE_3(3,4)+beta*SE_3(3,1)],'--r');

           plot3([SE_3(1,4),SE_3(1,4)+beta*SE_3(1,2)],[SE_3(2,4),SE_3(2,4)+beta*SE_3(2,2)],[SE_3(3,4),SE_3(3,4)+beta*SE_3(3,2)],'--g');
           plot3([SE_3(1,4),SE_3(1,4)+beta*SE_3(1,3)],[SE_3(2,4),SE_3(2,4)+beta*SE_3(2,3)],[SE_3(3,4),SE_3(3,4)+beta*SE_3(3,3)],'--b');
        end;    
%         for j = 1:14
%             cframep(:,:) =  inv(SE_3) *  y(:,:);
%             if cframep(1,j) >= focall & cframep(1,j) <= focalh & abs(cframep(2,j)) <= cframep(1,j)*tan(hov/2) & abs(cframep(3,j)) <= cframep(1,j)*tan(vov/2);
%                     plot3(y(1,j),y(2,j),y(3,j),'o','Color','r','MarkerSize',5);   
%             else 
%                     plot3(y(1,j),y(2,j),y(3,j),'o','Color','b','MarkerSize',5);
%             end;
% 
%         end;
%       hold off;
%       pause(0.01)
end