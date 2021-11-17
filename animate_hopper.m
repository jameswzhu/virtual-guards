% animate_hopper: generate animation of hopper system motion
% James Zhu
% Carnegie Mellon University Robomechanics Lab
function animate_hopper(state_vec)

bRecord = 1;  % Uncomment this to save a video
if bRecord
    % Define video recording parameters
    v = VideoWriter('unstable_slip', 'MPEG-4');
    v.Quality = 100;
    v.FrameRate = 60;
    open(v);
end

% Create trace of trajectory and particle object
particle = [];
paddle_handle = [];
spring_handle = [];

% Set up axes
axis equal
ylabel('y')

% draw
for ii = 1:50:length(state_vec)
    
    x = state_vec(ii,1);
    % Define axis window
    xmin = x - 1.5;
    xmax = x + 1.5;
    ymin = -0.1;
    ymax = 3;
    axis([xmin xmax ymin ymax])
    
    a = tic;
%     addpoints(h,xout(ii,1),xout(ii,2));
    drawnow limitrate
    delete(particle) % Erases previous particle
     
    a1 = 0;
    particle = circles(x,state_vec(ii,3), 0.1, 'color', [1 0 0]);
    
    hold on;
    delete(paddle_handle) % Erases previous particle
    paddle_handle = plot([xmin,xmax],a1*ones(1,2), 'k');
    
    delete(spring_handle) % Erases previous particle
    spring_handle = plot([x,state_vec(ii,5)],[state_vec(ii,3),state_vec(ii,6)], 'k'); 
    
%     paddle = contour(X,Y,a1,[0,0], 'k'); hold on;
    if bRecord
        frame = getframe(gcf);
        writeVideo(v,frame);
    else
%         pause(dt - toc(a)); % waits if drawing frame took less time than anticipated
            pause(0.01);
    end
end

if bRecord
    close(v);
end