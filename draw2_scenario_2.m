
t = out.STATES(:,2);
px = out.STATES(:,3);
py = out.STATES(:,4);

obx = out.STATES(:,9);
oby = out.STATES(:,10);

targetx = out.STATES(:,11);
targety = out.STATES(:,12);

hold on
axis([-18,18,-12,12])
pause(2)
do = linspace(0,2*3.14);
dox = sqrt(2*4)*cos(do);
doy = sqrt(2*4)*sin(do);
plot (dox,doy,'k');

for i  = 1:1:length(t)
    dw = linspace(0,2*3.14);
    dw_ox = obx(i)+sqrt(1.3)*cos(dw);
    dw_oy = oby(i)+sqrt(1.3)*sin(dw);
    
    dw_tx = targetx(i)+.5*cos(dw);
    dw_ty = targety(i)+.5*sin(dw);
    
    h1 = plot (dw_ox,dw_oy,'r');
    h2 = plot (dw_tx,dw_ty,'b');
    
    h3 = plot (px(i),py(i),'r*');
    pause(10/64);
    if i ~= length(t)
        set(h1,'Visible','off');
        set(h2,'Visible','off');
        set(h3,'Visible','off');
    end
end