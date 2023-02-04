clear
clc

x = [0,0.1,1,1.5,2,2.9,3];

v=[0,0,3,3,3,1,1];

step = 10;
k_xq=3/step;

xq= 0:k_xq:3;

figure
vq1 = interp1(x,v,xq,'linear');
plot(x,v,'o',xq,vq1,':.');
xlim([0 3]);
title('(Default) Linear Interpolation');
hold on
grid on

%%

dvq1 = vq1;

for i=1:1:length(vq1)
    if i==1
        vq1_=0;
    else
        vq1_=vq1(i-1);
    end
    dvq1(i) = (vq1(i)-vq1_)/k_xq;
end

plot(xq,dvq1,'--');

%%
% 
% ddv = vq1;
% 
% for i=1:1:length(vq1)
%     if i==1
%         dvq1_= 0;
%     else
%         dvq1_= vq1(i-1);
%     end
%     ddv(i) = (dvq1(i)-dvq1_)/k_xq;
% end
% 
% plot(xq,ddv,'*-');
