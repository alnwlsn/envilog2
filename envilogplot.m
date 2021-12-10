clear;
clc;
clf;

raw = csvread("envilog-2021-12-05-indoor.csv");

%frame format:
%[ time, ds3231T, temp, pressure, humidity ]

i = 0;
while(i<length(raw(:,1)))
  i++;
  frame(i,:)=[datenum([2000+raw(i,3),raw(i,4),raw(i,5),raw(i,6),raw(i,7),raw(i,8)]),raw(i,9),raw(i,10),raw(i,11),raw(i,12)];
end

figure(1)
ha(1)=subplot(3,1,1);
hold on;
title(strcat("Enviroment data: ",datestr(frame(1,1)," mmm 'yy ")," to",datestr(frame(i,1)," mmm 'yy ")));
plot(frame(:,1),frame(:,2)*(9/5)+32);
plot(frame(:,1),frame(:,3)*(9/5)+32);
datetick("x", "mmm/yy");
grid on;
xlabel("Time");
ylabel("Temperature (F)");

ha(2)=subplot(3,1,2);
plot(frame(:,1),frame(:,4));
datetick("x", "mmm/yy");
grid on;
xlabel("Time");
ylabel("Pressure (Pa)");

ha(3)=subplot(3,1,3);
plot(frame(:,1),frame(:,5));
datetick("x", "mmm/yy");
grid on;
xlabel("Time");
ylabel("Humidity (%)");

linkaxes(ha, "x");