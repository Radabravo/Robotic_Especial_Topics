
    totalTime=10;
    timeInterval=0.5;
    taFactor=0.2;
    tajFactor=0.25;
    velLi1near=1;
    %%Geração de um perfil de velocidade trapezoidal
    % Dado um tempo de t segundos
    t = 0:timeInterval:totalTime;
    v=zeros(1,length(t));
    acc=zeros(1,length(t));
%     % Uma trajetória retilinea de x metros
%     dt=totalDistance;
    %Temos uma velocidade media vavg
    vavg=velLi1near;
    
    %Considerando ta e td como 20% do tempo total
    ta=taFactor*t(end);
    td=ta;
    tc=1-ta-td;
    
    %Considerando o perfil de aceleração como triangular, temos os tempos de
    %jerk em 50% da aceleração
    taj=ta*tajFactor;
    tdj=taj;
    tcj=ta-taj-tdj;
    
    %tc sendo o tempo de velocidade constante
    tc=t(end)-ta-td;
    %dado dt=((1/2)*vmax)*(ta+td)+vmax*(tc)
    %temos dt=(1/10)*vmax+vmax*(8/10) e dt=vavg*tt
    vmax=vavg*((ta+tc)/t(end))^-1;
    accavg=vmax/(ta);
    accmax=accavg*((taj+tcj)/ta)^-1;
    jerk=accmax/taj;
    i=2;
    
    while(t(i)<=taj)
        
        acc(i)=acc(i-1)+jerk*(t(i)-t(i-1));
        i=i+1;
        
    end
    while(t(i)<=(taj+tcj))
        
        acc(i)=acc(i-1);
        i=i+1;
        
    end
    while(t(i)<=(taj+tcj+tdj))
        
        acc(i)=acc(i-1)-jerk*(t(i)-t(i-1));
        i=i+1;
        
    end
    while(t(i)<=(taj+tcj+tdj+tc))
        
        acc(i)=acc(i-1);
        i=i+1;
        
    end
    while(t(i)<=(taj+tcj+tdj+tc+tdj))
        
        acc(i)=acc(i-1)-jerk*(t(i)-t(i-1));
        i=i+1;
        
    end
    while(t(i)<=(taj+tcj+tdj+tc+taj+tcj))
        
        acc(i)=acc(i-1);
        i=i+1;
        
    end
    while(t(i)<(taj+tcj+tdj+tc+taj+tcj+tdj))
        
        acc(i)=acc(i-1)+jerk*(t(i)-t(i-1));
        i=i+1;
        
    end
    for i=2:length(t)
        v(i)=v(i-1)+acc(i)*(t(i)-t(i-1));
    end
    
    d=zeros(1,length(v));
    
    for i=2:length(v)
        d(i)=d(i-1)+(v(i))*(t(i)-t(i-1));
    end
    
    figure('Name','Profiles');
    subplot(3,1,1)
    plot(t,acc)
    title('S-curve Acc')
    subplot(3,1,2)
    plot(t,v)
    title('S-curve Velocity')
    subplot(3,1,3)
    plot(t,d)
    title('S-curve trajectory')
