x = [60 65 70 80 90 100 110 120];
y = [0.018 0.066 0.1 0.209 0.266 0.347 0.499 0.589 ];
coef1 = polyfit(x,y,1)
plot1 = polyval(coef1, x);
x2 = [60 65 70 80 90 100 110 120];
y2 = [0 0 0.1 0.209 0.266 0.347 0.499 0.589 ];
coef2 = polyfit(x2,y2,1)
plot2 = polyval(coef2, x2);



figure('Name','Interpolation');
subplot(2,1,1)
plot(x,plot1)
title('With 60 and 65 pwm')
subplot(2,1,2)
plot(x2,plot2)
title('Without 60 and 65 pwm')
