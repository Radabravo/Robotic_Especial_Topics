%%Geração de um perfil de velocidade trapezoidal, visando a diminuição de
%%trancos
% Dado um tempo de t segundos
t = 0:0.5:10;
v=zeros(1,length(t));
% Uma trajetória retilinea de x metros
dt=1;
%Temos uma velocidade media vavg
vavg=dt/t(end);
%Considerando ta e td como 10% do tempo total
ta=0.10*t(end);
td=ta;
tc=t(end)-ta-td;
%dado dt=((1/2)*vmax)*(ta+td)+vmax*(tc)
%temos dt=(1/10)*vmax+vmax*(8/10) e dt=vavg*tt
vmax=vavg*10/9;
a=vmax/(ta);
i=2;
while(t(i)<=ta)
    v(i)=v(i-1)+a*(t(i)-t(i-1));
    i=i+1;
end
while(t(i)<=(ta+tc))
    v(i)=v(i-1);
    i=i+1;
end
while(t(i)<(ta+tc+td))
    v(i)=v(i-1)-a*(t(i)-t(i-1));
    i=i+1;
end

d=zeros(1,length(v));

for i=2:length(v)
    d(i)=d(i-1)+(v(i))*(t(i)-t(i-1));
end

figure('Name','Profiles');
subplot(2,1,1)
plot(t,v)
title('Trapezoidal Velocity')
subplot(2,1,2)
plot(t,d)
title('S trajectory')