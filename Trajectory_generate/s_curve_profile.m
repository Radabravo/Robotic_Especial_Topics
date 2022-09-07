%%Geração de um perfil de velocidade trapezoidal
% Dado um tempo de t segundos
t = 0:0.001:10;
v=zeros(1,length(t));
acc=zeros(1,length(t));
% Uma trajetória retilinea de x metros
dt=5;
%Temos uma velocidade media vavg
vavg=dt/t(end);

%Considerando ta e td como 10% do tempo total
ta=0.10*t(end);
td=ta;

%Considerando o perfil de aceleração como triangular, temos os tempos de
%jerk em 50% da aceleração
taj=ta/2;
tdj=taj;

%tc sendo o tempo de velocidade constante
tc=t(end)-ta-td;
%dado dt=((1/2)*vmax)*(ta+td)+vmax*(tc)
%temos dt=(1/10)*vmax+vmax*(8/10) e dt=vavg*tt
vmax=vavg*10/9;
accavg=vmax/(ta);
accmax=2*accavg;
jerk=accmax/taj;
i=2;

while(t(i)<=taj)
    acc(i)=acc(i-1)+jerk*(t(i)-t(i-1));
    i=i+1;
end
while(t(i)<=(taj+tdj))
    acc(i)=acc(i-1)-jerk*(t(i)-t(i-1));
    i=i+1;
end

while(t(i)<=ta)
    v(i)=v(i-1)+accavg*(t(i)-t(i-1));
    i=i+1;
end
while(t(i+1)<=(ta+tc))
    acc(i)=acc(i-1);
    i=i+1;
end

while(t(i+1)<=(ta+tc))
    v(i)=v(i-1);
    i=i+1;
end
while(t(i+1)<(ta+tc+tdj))
    acc(i)=acc(i-1)-jerk*(t(i)-t(i-1));
    i=i+1;
end
while(t(i+1)<(ta+tc+tdj+tdj))
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