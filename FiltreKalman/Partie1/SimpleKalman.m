close all;
global realpos;
global t;
endtime = 100;
time = linspace(1,endtime, 1000);
realpos = sin(time/5);
plot(time, realpos);
hold on;

%Estimation
processNoise = 0.1;
measurementNoise = 0.1; 

x = realpos(1);
p = processNoise;
unfilteredOdometry = x;
for t = 1:length(time)

  %-------------------- "Prediction Step" --------------------
  u = takeOdometry();
  x = x + u;
  p = p + processNoise;

  %-------------------- "Update Step" --------------------
  if mod(t, 10) == 0
    z = takeMeasurement();
    plot(time(t), z, 'r*');

    y = z - x;                         

    k = p / (p + measurementNoise);   

    x = x + k*y;                       
                         
    

    p = (1 - k)*p;                    
    disp(p)
  end

  plot(time(t), x, 'k.');


  unfilteredOdometry = unfilteredOdometry + u;
  plot(time(t), unfilteredOdometry, 'go');

  pause(0.01);
end
