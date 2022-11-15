    totalTime=10;
    timeInterval=0.5;
    taFactor=0.1;
    velLi1near=0.1;
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
    
    
    
    %tc sendo o tempo de velocidade constante
    tc=t(end)-ta-td;
    %dado dt=((1/2)*vmax)*(ta+td)+vmax*(tc)
    %temos dt=(1/10)*vmax+vmax*(8/10) e dt=vavg*tt
    vmax=vavg*((ta+tc)/t(end))^-1;
    accavg=vmax/(ta);       
    i=1;
    while(t(i)<ta)
        
        acc(i)=accavg;
        i=i+1;
        
    end
    
    while(t(i)<=(ta+tc))
        
        acc(i)=0;
        i=i+1;
        
    end
    while(t(i)<(ta+td+tc))
        
        acc(i)=-accavg;
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