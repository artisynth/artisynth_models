per = 1;
dur = 10;
amp = 6000;
ampSlope = 170;
dt = .1;
fp = fopen('lungpressureIP.txt','w');
fprintf(fp,'2000000000 20000000000 1.0\n');
fprintf(fp,'linear 1 explicit\n');

N = floor(dur/dt);
t = 0;
slope = amp/per;
p=amp
for i=0:N
  p = p - slope * dt;
  amp = amp - ampSlope * dt;
  if(p<0)
    p = amp;
    slope = amp/per;
  end
  fprintf(fp,'%f %f\n',t,p);
  t = t + dt;
end

fclose(fp);