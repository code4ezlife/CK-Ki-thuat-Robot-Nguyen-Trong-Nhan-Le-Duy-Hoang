function animate_run(scn, res, cfg, outputFile)
%ANIMATE_RUN Ghi video MP4 cho mot lan mo phong robot.
%   animate_run(scn, res, cfg, outputFile)
%   Hien thi ban do, duong A*, quy dao robot, nguoi di bo va cac diem replan.
%   Video duoc ghi truc tiep, khong can mo cua so figure.

if nargin < 4 || isempty(outputFile)
    outputFile = fullfile('videos', [upper(res.algo) '.mp4']);
end
[outDir,~,~] = fileparts(outputFile);
if ~isempty(outDir) && ~exist(outDir, 'dir')
    mkdir(outDir);
end

try
    vw = VideoWriter(outputFile, 'MPEG-4');
catch ME
    error('Khong the tao MP4 bang VideoWriter. Hay dung MATLAB co ho tro MPEG-4. Chi tiet: %s', ME.message);
end
vw.FrameRate = 30;
 vw.Quality = 95;
open(vw);
videoCleanup = onCleanup(@() close(vw)); %#ok<NASGU>

fig = figure('Name', ['Video - ' upper(res.algo)], 'Color','w', ...
    'Visible','on', 'Position',[100 100 1280 720]);
figCleanup = onCleanup(@() close(fig)); %#ok<NASGU>
ax = axes('Parent',fig);
axes(ax); %#ok<LAXES>
draw_map(scn.map);
hold(ax,'on');

% Duong di A* ban dau va dich den.
if isfield(res,'path0') && ~isempty(res.path0)
    plot(ax,res.path0(:,1),res.path0(:,2),'k--','LineWidth',1.2,'DisplayName','A* path');
elseif isfield(res,'path_final') && ~isempty(res.path_final)
    plot(ax,res.path_final(:,1),res.path_final(:,2),'k--','LineWidth',1.2,'DisplayName','Planned path');
end
plot(ax,scn.start(1),scn.start(2),'go','MarkerFaceColor','g','MarkerSize',9,'DisplayName','Start');
plot(ax,scn.goal(1),scn.goal(2),'rp','MarkerFaceColor','r','MarkerSize',14,'DisplayName','Goal');

% Duong di du kien sau khi tinh lai (S5).
if isfield(res,'path_final') && ~isempty(res.path_final)
    plot(ax,res.path_final(:,1),res.path_final(:,2),':','Color',[0.35 0.35 0.35], ...
        'LineWidth',1.4,'DisplayName','Final A* path');
end
if isfield(res,'replan_pos') && ~isempty(res.replan_pos)
    plot(ax,res.replan_pos(:,1),res.replan_pos(:,2),'kx','MarkerSize',9, ...
        'LineWidth',1.8,'DisplayName','Replan');
end

% Ve vet di chuyen that cua tung nguoi (neu kich ban co nguoi).
nH = 0;
if isfield(res,'hum_true') && ~isempty(res.hum_true)
    nH = size(res.hum_true,2);
end
humanColors = lines(max(nH,1));
humanTrail = gobjects(nH,1);
humanBody = gobjects(nH,1);
th = linspace(0,2*pi,40);
for j = 1:nH
    hp = reshape(res.hum_true(:,j,:), size(res.hum_true,1), 2);
    valid = all(isfinite(hp),2);
    humanTrail(j) = plot(ax,nan,nan,'-','Color',humanColors(j,:), ...
        'LineWidth',1.3,'DisplayName',sprintf('Person %d trail',j));
    humanBody(j) = plot(ax,nan,nan,'o','Color',humanColors(j,:), ...
        'MarkerFaceColor',humanColors(j,:),'MarkerSize',9, ...
        'DisplayName',sprintf('Person %d',j));
end

% Robot va vet quy dao.
robotBody = fill(ax,res.x(1)+cfg.r_robot*cos(th),res.y(1)+cfg.r_robot*sin(th), ...
    [0 0.45 0.74],'EdgeColor',[0 0.2 0.45],'LineWidth',1.2,'DisplayName','Robot');
robotTrail = plot(ax,res.x(1),res.y(1),'b-','LineWidth',2,'DisplayName','Robot trajectory');
heading = plot(ax,[res.x(1) res.x(1)+cfg.r_robot*cos(res.th(1))], ...
    [res.y(1) res.y(1)+cfg.r_robot*sin(res.th(1))],'Color',[0 0.2 0.45], ...
    'LineWidth',2,'DisplayName','Heading');

axis(ax,'equal');
xl = xlim(ax); yl = ylim(ax);
set(ax,'FontSize',12,'LineWidth',1.0);
xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
legend(ax,'Location','eastoutside');

nFrames = numel(res.t);
% Giu dung thoi luong mo phong: moi buoc dt duoc lap frame cho du 30 FPS.
framesPerStep = max(1,round(vw.FrameRate*cfg.dt));
for k = 1:nFrames
    set(robotBody,'XData',res.x(k)+cfg.r_robot*cos(th), ...
        'YData',res.y(k)+cfg.r_robot*sin(th));
    set(robotTrail,'XData',res.x(1:k),'YData',res.y(1:k));
    set(heading,'XData',[res.x(k) res.x(k)+cfg.r_robot*cos(res.th(k))], ...
        'YData',[res.y(k) res.y(k)+cfg.r_robot*sin(res.th(k))]);

    if nH > 0
        hk = min(k,size(res.hum_true,1));
        for j = 1:nH
            hp = reshape(res.hum_true(:,j,:),size(res.hum_true,1),2);
            last = hp(hk,:);
            if all(isfinite(last))
                valid = all(isfinite(hp(1:hk,:)),2);
                trail = hp(1:hk,:);
                trail = trail(valid,:);
                set(humanTrail(j),'XData',trail(:,1),'YData',trail(:,2));
                set(humanBody(j),'XData',last(1),'YData',last(2));
            else
                set(humanBody(j),'XData',nan,'YData',nan);
            end
        end
    end

    title(ax,sprintf('%s | %s | t = %.1f s | %d/%d', ...
        scn.name,upper(res.algo),res.t(k),k,nFrames),'Interpreter','none');
    xlim(ax,xl); ylim(ax,yl);
    drawnow;
    frame = getframe(fig);
    for repeat = 1:framesPerStep
        writeVideo(vw,frame);
    end
end

fprintf('Da xuat video: %s\n',outputFile);
end
