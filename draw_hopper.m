% animate_hopper: generate animation of hopper system motion
% James Zhu
% Carnegie Mellon University Robomechanics Lab
function draw_hopper(state_vec,alpha)
    
x = state_vec(1);

a = tic;
%     addpoints(h,xout(ii,1),xout(ii,2));
drawnow limitrate

a1 = 0;
particle = circles(x,state_vec(3), 0.09, 'color', [1 0 0],'facealpha',alpha,'edgealpha',alpha);

hold on;
spring_handle = plot([x,state_vec(5)],[state_vec(3),state_vec(6)], 'color',[0,0,0,alpha]); 
uistack(spring_handle,'bottom')