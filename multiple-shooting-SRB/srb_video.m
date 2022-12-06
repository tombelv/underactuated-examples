function srb_video(X, U, traj, params, suff_str) 
pause(1)



% Repeat the last input allowing to render the last frame
U = [U U(:,end)];

% Number of frames to skip for each rendered frame
skip = 1;

% Length of rectangle (long edge)
L = 0.6;
% Aspect ratio of the rectangle (long_edge/short_edge)
AR = 3/2;

W = 0.2; % this has to be taken from the params eventually

srb = polyshape([-L/2*(1/AR) -L/2; ...
                  L/2*(1/AR) -L/2; ...
                  L/2*(1/AR)  L/2; ...
                 -L/2*(1/AR)  L/2]);

foot = polyshape([-W/2     0; ...
                   W/2     0; ...
                   W/2  0.03; ...
                  -W/2  0.03]);



dimvideo = [900 900];
figure('units','pixels','Position', [100 50 dimvideo])




 
v = VideoWriter("SRB_anim_" + suff_str);
v.FrameRate = 1/params.delta/skip;
open(v);


for i=1:skip:(length(X) + skip*v.FrameRate)

    clf

    n = min(length(X),i);
    
    hold on
    
    srb_rot = rotate(srb, X(5,n)*180/pi);
    srb_trans = translate(srb_rot, X([1,3], n)');

    plot(srb_trans)
    plot(X(1,n), X(3,n), 'Marker', 'o', 'LineWidth', 8.0, 'Color', 'k')

    plot(foot)

    plotArrow([W/2, 0], [W/2+U(3,n)/300, U(4,n)/300])
    plotArrow([-W/2, 0], [-W/2+U(1,n)/300, U(2,n)/300])

    line([X(1,n),X(1,n)], [0, X(3,n)], 'LineStyle', '--', 'Color', 'k')
    
        
    axis(gca,'equal')
    xlim([-1 1])
    ylim([-0.2 1.8])
    grid on; hold on
    xlabel("x [m]")
    ylabel("z [m]")
    title("$t=$ "+num2str(params.delta*(n-1),'%.2f')+" s", 'FontSize', 18, 'Interpreter', 'latex')

    legend(["SRB", "CoM", "SUPPORT POLYGON", "Ground reaction forces"])


    set(gcf,'units','pixels','position',[100 50 dimvideo])
    FF = getframe(gcf);
    writeVideo(v,FF);

%     pause

end


close(v);


end

function plotArrow(start_pos, tip_pos)

    line([start_pos(1), tip_pos(1)],[start_pos(2), tip_pos(2)], 'Color','r', 'LineWidth',3.0')

    plot(adaptArrowPoint(start_pos, tip_pos), 'EdgeColor', 'r', 'FaceColor','r', 'FaceAlpha', 1)


end


function arrow_out = adaptArrowPoint(start_pos, tip_pos)

arrow_point = polyshape([0   -0.05; ...
                         0.2    0; ...
                         0    0.05]);

min_scale = 0.2;
max_scale = 0.4;

arrow_point_scaled = scale(arrow_point, max(min_scale,min(norm(tip_pos-start_pos),max_scale)) , [0,0]);
arrow_point_rotated = rotate(arrow_point_scaled,atan2d(tip_pos(2)-start_pos(2),tip_pos(1)-start_pos(1)));
arrow_point_translated = translate(arrow_point_rotated,tip_pos);

arrow_out = arrow_point_translated;

end