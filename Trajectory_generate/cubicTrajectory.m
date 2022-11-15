    
    totalTime=10;
    timeInterval=0.5;
    t=0:timeInterval:totalTime;
    initialS=0;
    finalS=1;
    d=zeros(1,length(t));
    v=zeros(1,length(t));
    acc=zeros(1,length(t));
    a0=initialS
    a1=0;
    a2=3*(finalS-initialS)/totalTime^2;
    a3=-2*(finalS-initialS)/totalTime^3;
    disp = a0 + a1.*t + a2.*t.^2 +a3*t.^3;
    vel = a1 + 2*a2.*t +3*a3*t.^2;
    acc = 2*a2 + 6*a3.*t;


    for i=2:length(t)
        v(i)=v(i-1)+acc(i)*(timeInterval);
    end    
    
    for i=2:length(v)
        d(i)=d(i-1)+(v(i))*(t(i)-t(i-1));
    end
    figure('Name','Profiles discrete');
    subplot(3,1,1)
    plot(t,acc)
    title('Acc')
    subplot(3,1,2)
    plot(t,v)
    title('Velocity')
    subplot(3,1,3)
    plot(t,d)
    title('Trajectory')

    figure('Name','Profiles continous');
    subplot(3,1,1)
    plot(t,acc)
    title('Acc')
    subplot(3,1,2)
    plot(t,vel)
    title('Velocity')
    subplot(3,1,3)
    plot(t,disp)
    title('Trajectory')
