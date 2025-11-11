function p_end = circleDrawer(p_center, R, angles)

    % Angles = range of angles in radians to sweep by the circle
    % R = radius
    % p_center = center of the circle

    p_end = [];
    
    for i = 1:length(angles)

        p_end(:, end + 1) = p_center + R * [0; cos(angles(i)); sin(angles(i))]; 

    end

    % ACTIVATE THIS FOR CHECKING

    % plot(p_end(2, :), p_end(3, :))
    % axis equal

end 