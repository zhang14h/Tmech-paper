function mthetatrix = therotation(fei,theta,miu)
      
      
      
      
      mthetatrix   = [cos(fei) -sin(fei) 0; sin(fei) cos(fei) 0;0 0 1 ]*[cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)]*[1 0 0;0 cos(miu) -sin(miu);0 sin(miu) cos(miu)];
              
end

      
      
     