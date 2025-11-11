%% Robot animation 

function animate_robot(poses, fkfun)
    % poses: N×4 [th1 th2 th3 th4] (rad or deg — just be consistent with fkfun)
    % fkfun: @(th) -> [p0; p1; p2; p3; p4], 5×3 joint positions in {0}

    % Prepare figure and links
    clf; hold on; grid on; axis equal
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(45,25);
    axis([-250 250 -250 250 0 250]);  % in mm

    
    hLinks = plot3(nan,nan,nan,'-o','LineWidth',2,'MarkerSize',6);
    hTip   = plot3(nan,nan,nan,'ro','MarkerFaceColor','r','MarkerSize',6);  % end-effector tip
    
    trail = [];
    for k = 1:size(poses,1)
        P = fkfun(poses(k,:));
        trail = [trail; P(end,:)];  % accumulate tip path
        set(hLinks,'XData',P(:,1),'YData',P(:,2),'ZData',P(:,3));
        set(hTip,'XData',P(end,1),'YData',P(end,2),'ZData',P(end,3));
        plot3(trail(:,1),trail(:,2),trail(:,3),'.','Color',[0 0.5 1]);  % leave visible path
        pause(0.5)
        drawnow;
      

       
    end

    

end



function demo_visualize(poses)
    % ---- edit these lengths (meters) to match your arm ----
    d1 = 50;   % base to joint1 along z
    L2 = 93;   % link 2 length
    L3 = 93;   % link 3 length
    L4 = 50;   % wrist link (end-effector tip at frame4)
    % -------------------------------------------------------

    fkfun = @(th) fk4(th, d1, L2, L3, L4);
    animate_robot(poses, fkfun);
end

function P = fk4(th, d1, L2, L3, L4)
    % th = [t1 t2 t3 t4] (radians)
    t1 = th(1); t2 = th(2); t3 = th(3); t4 = th(4);

    % Define DH transform inline
    DH = @(theta, d, a, alpha) [ ...
        cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
        0,           sin(alpha),             cos(alpha),             d;
        0,           0,                      0,                      1];

    % your DH parameters
    T01 = DH(t1, d1, 0, pi/2);
    T12 = DH(t2 + pi/2, 0, L2, 0);
    T23 = DH(t3, 0, L3, 0);
    T34 = DH(t4, 0, L4, 0);

    % chain them
    T0 = eye(4);
    T1 = T0 * T01;
    T2 = T1 * T12;
    T3 = T2 * T23;
    T4 = T3 * T34;

    % extract joint positions
    p0 = T0(1:3,4).';
    p1 = T1(1:3,4).';
    p2 = T2(1:3,4).';
    p3 = T3(1:3,4).';
    p4 = T4(1:3,4).';

    P = [p0; p1; p2; p3; p4];
end




load("poses.mat")   

demo_visualize(poses)
