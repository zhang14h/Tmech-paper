a = imread('C1 l.png');
b = imread('C1 eta.png');
c = imread('C1 K_o.png');

figure('position',[0 0 1000 800]);
subplot 311;imshow(a);title('Test 1 for Case 1');
subplot 312;imshow(b);title('Test 2 for Case 1');
subplot 313;imshow(c);title('Test 3 for Case 1');