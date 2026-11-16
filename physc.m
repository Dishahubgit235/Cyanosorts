%% PHYSICAL AI : "SEE AND ACT"  (3D TUBE ARM + TWO-FINGER GRIPPER + CONVEYOR)
%  Language -> Conveyor -> Camera -> Perception -> Decision -> 3D IK -> Pick & Place
%  Requires: base MATLAB only (R2019b+ for tiledlayout, R2018b+ for yline/sgtitle).

clear; clc; close all; rng(7);

%% ---------------- 1. CONFIGURATION ----------------
cfg.L1 = 0.35;  cfg.L2 = 0.30;                 % link lengths (m)
cfg.H  = 0.25;                                 % shoulder height above table (m)
cfg.hg = 0.10;                                 % wrist-to-fingertip length (m)
cfg.zHigh = 0.22;                              % gripper travel height (m)
cfg.xlim = [-0.7 0.7];  cfg.ylim = [-0.5 0.7]; % camera field of view (m)
cfg.ppm  = 150;                                % camera pixels per metre
cfg.imgW = round(diff(cfg.xlim)*cfg.ppm);
cfg.imgH = round(diff(cfg.ylim)*cfg.ppm);
cfg.rObj = 0.03;                               % object radius (m)
cfg.noise = 0.02;                              % camera noise (std dev)
cfg.minArea = 25;                              % min blob size (pixels)
cfg.tol = 0.015;                               % gripper tolerance (m)
cfg.slipProb = 0.25;                           % chance a grasp slips -> shows retry
cfg.fr = 0.010;                                % finger radius (m)
cfg.wOpen   = 0.065;                           % finger half-opening, open (m)
cfg.wClosed = cfg.rObj + cfg.fr;               % finger half-opening, closed on ball
cfg.homeQ = [pi/2, deg2rad(70), deg2rad(-110)];% [base-yaw shoulder elbow]

% ---- conveyor settings ----
cfg.beltY     = [0.30 0.50];     % belt edges in y (m)
cfg.beltSpeed = 0.20;            % belt speed (m/s)  <-- raise for faster belt
cfg.stopX     = 0.00;            % stop line: belt stops when front ball reaches this x
cfg.spawnX0   = -0.55;           % x of the first ball on the belt
cfg.spacing   = 0.22;            % distance between balls on the belt (m)
cfg.pickXlim  = [-0.30 0.30];    % robot reach zone along the belt (m)

% ---- colours (edit freely, [R G B] from 0 to 1) ----
cfg.colUpper  = [0.95 0.45 0.10];              % upper arm tube
cfg.colFore   = [0.15 0.45 0.85];              % forearm tube
cfg.colJoint  = [0.25 0.25 0.28];              % joints + pedestal
cfg.colGrip   = [0.40 0.40 0.45];              % gripper palm + fingers
cfg.colOpen   = [0.00 0.75 0.00];              % finger pads when open
cfg.colClosed = [0.90 0.00 0.00];              % finger pads when closed

% ---- speed settings ----
cfg.moveTime   = 2.0;    % seconds per big arm move
cfg.pauseSee   = 1.0;    % seconds to look at camera detection
cfg.pauseStep  = 0.5;    % seconds pause at each stage

names = {'Red','Green','Blue'};
cols  = [0.90 0.10 0.10; 0.10 0.80 0.10; 0.10 0.20 0.95];
bins  = [-0.52 0.12;  0.00 -0.42;  0.52 0.12];  % bin centres for R, G, B
nPerColor = 2;

%% ---------------- 1b. NATURAL-LANGUAGE COMMAND ----------------
disp('Examples: "sort everything" | "pick only blue" | "put red in the blue bin"');
disp('          "sort red first then blue" | "move all balls to green bin"');
cmd  = input('Command: ','s');
task = parseCommand(cmd, names);
fprintf('[ROBOT] Understood: handle %s | bins: %s\n', ...
    strjoin(names(task.order),' -> '), strjoin(names(task.binOf(task.order)),', '));

%% ---------------- 2. SPAWN OBJECTS ON THE BELT ----------------
objs = spawnObjects(nPerColor, cfg);

%% ---------------- 3. SET UP FIGURE (3D WORLD) ----------------
G.cfg = cfg;
[G.sx,G.sy,G.sz] = sphere(20);                 % unit sphere used for joints/balls
G.th = linspace(0,2*pi,40);                    % circle angles
fig = figure('Color','w','Position',[60 60 1350 620],'Name','Physical AI - See and Act (3D + Conveyor)');
tl = tiledlayout(fig,1,2,'TileSpacing','compact');
G.axCam = nexttile(tl); title(G.axCam,'ROBOT EYE  (camera + detection)');
G.ax = nexttile(tl); hold(G.ax,'on');
axis(G.ax,'equal'); axis(G.ax,[cfg.xlim cfg.ylim 0 0.8]); grid(G.ax,'on');
view(G.ax,-25,28);
xlabel(G.ax,'x (m)'); ylabel(G.ax,'y (m)'); zlabel(G.ax,'z (m)');
light(G.ax,'Position',[-1 -1.5 2],'Style','infinite');
light(G.ax,'Position',[1.5 -0.5 1],'Style','infinite','Color',[0.5 0.5 0.5]);
try, sgtitle(fig, ['Command: "' cmd '"'], 'FontWeight','bold'); catch, end

% table surface
patch(G.ax,'XData',[cfg.xlim(1) cfg.xlim(2) cfg.xlim(2) cfg.xlim(1)], ...
           'YData',[cfg.ylim(1) cfg.ylim(1) cfg.ylim(2) cfg.ylim(2)], ...
           'ZData',zeros(1,4),'FaceColor',[0.93 0.93 0.93],'EdgeColor','none', ...
           'FaceLighting','none');

% conveyor belt, moving stripes and side rails
bx = cfg.xlim;  by = cfg.beltY;
patch(G.ax,'XData',[bx(1) bx(2) bx(2) bx(1)],'YData',[by(1) by(1) by(2) by(2)], ...
      'ZData',0.003*ones(1,4),'FaceColor',[0.22 0.22 0.25],'EdgeColor','none', ...
      'FaceLighting','none');
G.hStripe = plot3(G.ax,NaN,NaN,NaN,'-','LineWidth',2,'Color',[0.55 0.55 0.60]);
[X,Y,Z] = stripeXYZ(cfg,0);
set(G.hStripe,'XData',X,'YData',Y,'ZData',Z,'UserData',0);
for yy = by
    [X,Y,Z] = tubeXYZ([bx(1) yy 0.012],[bx(2) yy 0.012],0.012);
    surf(G.ax,X,Y,Z,'FaceColor',[0.75 0.75 0.78],'EdgeColor','none','FaceLighting','gouraud');
end
text(G.ax,bx(1)+0.05,by(2)+0.06,0,'CONVEYOR  ->','FontWeight','bold');
plot3(G.ax,[cfg.stopX cfg.stopX],by,[0.006 0.006],'--','Color',[1 0.8 0],'LineWidth',2); % stop line

% bins
for k = 1:3
    patch(G.ax,'XData',bins(k,1)+[-1 1 1 -1]*0.08, ...
               'YData',bins(k,2)+[-1 -1 1 1]*0.08, ...
               'ZData',0.002*ones(1,4),'FaceColor',0.5+0.5*cols(k,:), ...
               'EdgeColor',cols(k,:),'LineWidth',2,'FaceLighting','none');
    text(G.ax,bins(k,1),bins(k,2)-0.11,0,[names{k} ' bin'],'HorizontalAlignment','center');
end

% robot pedestal, base disk and shoulder ball (static)
[X,Y,Z] = tubeXYZ([0 0 0.02],[0 0 cfg.H],0.045);
surf(G.ax,X,Y,Z,'FaceColor',cfg.colJoint,'EdgeColor','none','FaceLighting','gouraud');
[X,Y,Z] = tubeXYZ([0 0 0],[0 0 0.02],0.09);
surf(G.ax,X,Y,Z,'FaceColor',cfg.colJoint,'EdgeColor','none','FaceLighting','gouraud');
patch(G.ax,'XData',0.09*cos(G.th),'YData',0.09*sin(G.th), ...
      'ZData',0.02*ones(size(G.th)),'FaceColor',cfg.colJoint, ...
      'EdgeColor','none','FaceLighting','none');
hSho = mkSph(G.ax,cfg.colJoint);
setSph(hSho,G,[0 0 cfg.H],0.05);

% objects (3D balls) + shadows
G.hSh  = gobjects(numel(objs),1);
G.hObj = gobjects(numel(objs),1);
for i = 1:numel(objs)
    G.hSh(i)  = plot3(G.ax,objs(i).xy(1),objs(i).xy(2),0.007,'o','MarkerSize',12, ...
        'MarkerFaceColor',[0.40 0.40 0.43],'MarkerEdgeColor','none');
    G.hObj(i) = mkSph(G.ax,cols(objs(i).c,:));
    setSph(G.hObj(i),G,[objs(i).xy cfg.rObj],cfg.rObj);
end

% moving arm parts (tubes + joints) and gripper
G.hUp = mkTube(G.ax,cfg.colUpper);             % upper arm
G.hEl = mkSph(G.ax,cfg.colJoint);              % elbow joint
G.hFo = mkTube(G.ax,cfg.colFore);              % forearm
G.hG  = gobjects(8,1);                         % gripper parts
G.hG(1) = mkTube(G.ax,cfg.colGrip);            % palm bar
G.hG(2) = mkSph(G.ax,cfg.colGrip);             % palm end A
G.hG(3) = mkSph(G.ax,cfg.colGrip);             % palm end B
G.hG(4) = mkTube(G.ax,cfg.colGrip);            % finger A
G.hG(5) = mkTube(G.ax,cfg.colGrip);            % finger B
G.hG(6) = mkSph(G.ax,cfg.colOpen);             % finger pad A
G.hG(7) = mkSph(G.ax,cfg.colOpen);             % finger pad B
G.hG(8) = mkSph(G.ax,cfg.colJoint);            % wrist joint
G.hTgt = plot3(G.ax,NaN,NaN,NaN,'-','LineWidth',3,'Color',[1 0.6 0]);  % target ring
drawArm(G, fk3(cfg.homeQ,cfg), cfg.homeQ(1), cfg.wOpen);
set(G.hG(1),'UserData',cfg.wOpen);             % current jaw opening stored here
title(G.ax,'WORLD  (3D robot arm + conveyor)');
set(fig,'GraphicsSmoothing','off');
drawnow; pause(1.5);

%% ---------------- 4. MAIN BELT -> SEE -> THINK -> ACT LOOP ----------------
q = cfg.homeQ;
binCount = zeros(1,3);
pickLog = struct('color',{},'err_mm',{},'ok',{});
maxIter = 5*numel(objs);  it = 0;

while any([objs.inScene] & ismember([objs.c], task.order)) && it < maxIter
    it = it + 1;

    % ---- CONVEYOR: run belt until the front wanted ball reaches the stop line ----
    title(G.ax,'CONVEYOR moving... (stops when a wanted ball reaches the stop line)');
    objs = moveBelt(G, objs, task);
    title(G.ax,'CONVEYOR stopped');
    drawnow; pause(cfg.pauseStep);

    % ---- SEE ----
    title(G.ax,'SEEING: camera is capturing an image...');
    drawnow; pause(cfg.pauseStep);
    img = renderCamera(objs, cfg);
    det = detectObjects(img, cfg, numel(names));
    cla(G.axCam); image(G.axCam,img); axis(G.axCam,'image','off'); hold(G.axCam,'on');
    for d = 1:numel(det)
        plot(G.axCam,det(d).px(1),det(d).px(2),'w+','MarkerSize',14,'LineWidth',2);
        text(G.axCam,det(d).px(1)+12,det(d).px(2)-12,names{det(d).c}, ...
            'Color','w','FontWeight','bold','BackgroundColor','k');
    end
    title(G.axCam,sprintf('ROBOT EYE: %d objects detected',numel(det)));
    drawnow; pause(cfg.pauseSee);

    % keep only the balls inside the robot's reach zone
    if ~isempty(det)
        inZone = arrayfun(@(d) d.xy(1) >= cfg.pickXlim(1) && d.xy(1) <= cfg.pickXlim(2), det);
        det = det(inZone);
    end
    % keep only the colours the command asked for
    det = det(ismember([det.c], task.order));
    if isempty(det)
        fprintf('[ROBOT] No requested object in reach - waiting for the belt\n');
        continue;
    end

    % ---- THINK: command order first, then front-most ball (closest to stop line) ----
    title(G.ax,'THINKING: choosing target + solving 3D inverse kinematics...');
    drawnow; pause(cfg.pauseStep);
    rnk = arrayfun(@(d) find(task.order==d.c), det);
    [~,j] = max(-1000*rnk + arrayfun(@(d) d.xy(1), det));
    tgt = det(j);
    qHi = ik3([tgt.xy cfg.zHigh], cfg);      % hover above the object
    qLo = ik3([tgt.xy cfg.rObj],  cfg);      % gripper around the object
    if isempty(qHi) || isempty(qLo)
        title(G.ax,'Target unreachable - skipping'); drawnow; pause(cfg.pauseSee);
        continue;
    end
    set(G.hTgt,'XData',tgt.xy(1)+0.05*cos(G.th),'YData',tgt.xy(2)+0.05*sin(G.th), ...
        'ZData',0.008*ones(size(G.th)));
    drawnow; pause(cfg.pauseStep);
    % ground truth bookkeeping (only for evaluation, robot never uses it)
    cand = find([objs.inScene] & [objs.c]==tgt.c);
    [~,m] = min(arrayfun(@(i) norm(objs(i).xy - tgt.xy), cand));
    oi = cand(m);
    err = norm(objs(oi).xy - tgt.xy);
    err = err + (rand < cfg.slipProb)*0.03;          % simulated grasp slip
    dst = task.binOf(tgt.c);                         % destination bin from the command
    fprintf('[ROBOT] Target: %s ball at (%.2f, %.2f) -> %s bin\n', ...
        names{tgt.c}, tgt.xy(1), tgt.xy(2), names{dst});

    % ---- ACT: move above the object ----
    title(G.ax,sprintf('Moving above the %s object...',names{tgt.c}));
    moveArm(G, q, qHi, 0, cfg.moveTime);  q = qHi;
    pause(cfg.pauseStep);

    if err > cfg.tol                       % grasp would fail -> re-see
        fprintf('[ROBOT] Grasp slipped, re-scanning and retrying\n');
        title(G.ax,'Grasp MISSED - looking again');
        set(G.hTgt,'XData',NaN,'YData',NaN,'ZData',NaN);
        drawnow; pause(cfg.pauseSee);
        pickLog(end+1) = struct('color',names{tgt.c},'err_mm',1000*err,'ok',false); %#ok
        moveArm(G, q, cfg.homeQ, 0, cfg.moveTime);  q = cfg.homeQ;
        continue;
    end

    % ---- ACT: lower and grip ----
    title(G.ax,'Lowering gripper...');
    moveArm(G, q, qLo, 0, cfg.moveTime/2);  q = qLo;
    title(G.ax,sprintf('Gripping %s object',names{tgt.c}));
    set(G.hG([6 7]),'FaceColor',cfg.colClosed);         % pads turn red
    setGrip(G, q, cfg.wClosed, 0.8);                    % fingers CLOSE on the ball
    pause(cfg.pauseStep);

    % ---- ACT: lift ----
    title(G.ax,'Lifting object...');
    moveArm(G, q, qHi, oi, cfg.moveTime/2);  q = qHi;

    % ---- ACT: carry above the destination bin ----
    b   = bins(dst,:) + 0.035*[mod(binCount(dst),2)-0.5, 0];
    qbHi = ik3([b cfg.zHigh], cfg);
    qbLo = ik3([b cfg.rObj],  cfg);
    title(G.ax,sprintf('Carrying %s object to the %s bin',names{tgt.c},names{dst}));
    moveArm(G, q, qbHi, oi, cfg.moveTime);  q = qbHi;

    % ---- ACT: lower into bin and release ----
    title(G.ax,'Lowering into bin...');
    moveArm(G, q, qbLo, oi, cfg.moveTime/2);  q = qbLo;
    objs(oi).xy = b;  objs(oi).inScene = false;
    moveObj(G, oi, [b cfg.rObj]);
    binCount(dst) = binCount(dst) + 1;
    pickLog(end+1) = struct('color',names{tgt.c},'err_mm',1000*err,'ok',true); %#ok
    title(G.ax,sprintf('Released %s object in the %s bin',names{tgt.c},names{dst}));
    set(G.hG([6 7]),'FaceColor',cfg.colOpen);           % pads turn green
    setGrip(G, q, cfg.wOpen, 0.8);                      % fingers OPEN, ball released
    set(G.hTgt,'XData',NaN,'YData',NaN,'ZData',NaN);
    drawnow; pause(cfg.pauseStep);

    % ---- rise and return home ----
    moveArm(G, q, qbHi, 0, cfg.moveTime/2);  q = qbHi;
    title(G.ax,'Returning to home pose...');
    moveArm(G, q, cfg.homeQ, 0, cfg.moveTime);  q = cfg.homeQ;
    pause(cfg.pauseStep);
end

title(G.ax,'DONE - command completed');
fprintf('[ROBOT] Command completed\n');
cla(G.axCam); image(G.axCam,renderCamera(objs,cfg)); axis(G.axCam,'image','off');
title(G.axCam,'Camera view after task');
drawnow;

%% ---------------- 5. RESULTS ----------------
nWanted = sum(ismember([objs.c], task.order));
fprintf('\n===== PERFORMANCE REPORT =====\n');
fprintf('Command            : "%s"\n', cmd);
if ~isempty(pickLog)
    fprintf('Objects sorted     : %d / %d\n', sum([pickLog.ok]), nWanted);
    fprintf('Pick attempts      : %d  (success rate %.0f%%)\n', numel(pickLog), 100*mean([pickLog.ok]));
    fprintf('Mean vision error  : %.2f mm\n', mean([pickLog.err_mm]));
    fprintf('Max  vision error  : %.2f mm\n', max([pickLog.err_mm]));
    figure('Color','w'); bar([pickLog.err_mm]); hold on; yline(1000*cfg.tol,'r--','Gripper tolerance');
    xlabel('Pick attempt'); ylabel('Localisation error (mm)'); title('Vision accuracy per pick');
else
    fprintf('No picks were made.\n');
end

%% =============== LOCAL FUNCTIONS ===============

function task = parseCommand(cmd, names)   % simple language understanding
    cmd = lower(strtrim(cmd));  low = lower(names);
    task.binOf = 1:3;  task.order = 1:3;   % default: sort everything to matching bins
    pos = zeros(1,3);
    for k = 1:3
        p = strfind(cmd, low{k});
        if ~isempty(p), pos(k) = p(1); end
    end
    [~,ix] = sort(pos);  ix = ix(pos(ix) > 0);      % colours in the order spoken
    tok = regexp(cmd,'(red|green|blue)\s*(bin|box|basket)','tokens');
    if ~isempty(tok)                                % "... in the blue bin"
        d = find(strcmp(low, tok{end}{1}));
        src = ix(ix ~= d);
        if isempty(src) || contains(cmd,'all') || contains(cmd,'every')
            src = 1:3;
        end
        task.order = src;  task.binOf(src) = d;
    elseif ~isempty(ix)
        task.order = ix;
        if contains(cmd,'first') || contains(cmd,'then')
            task.order = [ix setdiff(1:3, ix)];     % then the remaining colours
        end
    end
end

function objs = spawnObjects(nPer, cfg)   % balls in a line on the belt, random colour order
    colorList = repmat(1:3,1,nPer);
    colorList = colorList(randperm(numel(colorList)));
    objs = struct('xy',{},'c',{},'inScene',{});
    for k = 1:numel(colorList)
        x = cfg.spawnX0 - (k-1)*cfg.spacing + 0.02*(rand-0.5);
        y = mean(cfg.beltY) + 0.08*(rand-0.5);
        objs(end+1) = struct('xy',[x y],'c',colorList(k),'inScene',true); %#ok
    end
end

function objs = moveBelt(G, objs, task)   % run belt until the front WANTED ball hits the stop line
    cfg  = G.cfg;
    live = find([objs.inScene]);
    wl   = live(ismember([objs(live).c], task.order));   % wanted balls only
    tClock = tic;  tLast = 0;
    while max(arrayfun(@(k) objs(k).xy(1), wl)) < cfg.stopX - 1e-9
        now = toc(tClock);  dt = now - tLast;  tLast = now;
        lead = max(arrayfun(@(k) objs(k).xy(1), wl));
        dx = min(cfg.beltSpeed*dt, cfg.stopX - lead);       % never overshoot the stop line
        for k = live
            objs(k).xy(1) = objs(k).xy(1) + dx;
            moveObj(G, k, [objs(k).xy cfg.rObj]);
        end
        ph = get(G.hStripe,'UserData') + dx;                % scroll the belt stripes
        [X,Y,Z] = stripeXYZ(cfg, ph);
        set(G.hStripe,'XData',X,'YData',Y,'ZData',Z,'UserData',ph);
        drawnow;
    end
end

function [X,Y,Z] = stripeXYZ(cfg, phase)   % cross-stripes on the belt, shifted by phase
    sp = 0.08;
    xs = cfg.xlim(1) + mod(phase,sp) + sp*(0:floor(diff(cfg.xlim)/sp));
    xs = xs(xs <= cfg.xlim(2));  n = numel(xs);
    X = reshape([xs; xs; nan(1,n)],1,[]);
    Y = reshape([cfg.beltY(1)*ones(1,n); cfg.beltY(2)*ones(1,n); nan(1,n)],1,[]);
    Z = 0.005*ones(size(X));
end

function img = renderCamera(objs, cfg)
    cols = [0.90 0.10 0.10; 0.10 0.80 0.10; 0.10 0.20 0.95];
    x = cfg.xlim(1) + ((1:cfg.imgW)-0.5)/cfg.ppm;
    y = cfg.ylim(2) - ((1:cfg.imgH)'-0.5)/cfg.ppm;
    [X,Y] = meshgrid(x,y);
    img = 0.8*ones(cfg.imgH,cfg.imgW,3);
    for k = 1:numel(objs)
        if ~objs(k).inScene, continue; end
        mask = (X-objs(k).xy(1)).^2 + (Y-objs(k).xy(2)).^2 <= cfg.rObj^2;
        for ch = 1:3
            t = img(:,:,ch); t(mask) = cols(objs(k).c,ch); img(:,:,ch) = t;
        end
    end
    img = min(max(img + cfg.noise*randn(size(img)),0),1);   % sensor noise
end

function det = detectObjects(img, cfg, nCol)
    det = struct('c',{},'xy',{},'px',{},'area',{});
    R = img(:,:,1); Gc = img(:,:,2); B = img(:,:,3);
    masks = {R>0.55 & Gc<0.45 & B<0.45, ...        % red
             Gc>0.55 & R<0.45 & B<0.45, ...        % green
             B>0.55 & R<0.45 & Gc<0.50};           % blue
    for c = 1:nCol
        L = labelBlobs(masks{c});
        for n = 1:max(L(:))
            [r,cc] = find(L==n);
            if numel(r) < cfg.minArea, continue; end
            cx = mean(cc);  cy = mean(r);            % centroid in pixels
            xw = cfg.xlim(1) + (cx-0.5)/cfg.ppm;     % pixel -> world
            yw = cfg.ylim(2) - (cy-0.5)/cfg.ppm;
            det(end+1) = struct('c',c,'xy',[xw yw],'px',[cx cy],'area',numel(r)); %#ok
        end
    end
end

function L = labelBlobs(M)   % simple 8-connected component labelling
    [H,W] = size(M);  L = zeros(H,W);  n = 0;
    for idx = find(M)'
        if L(idx) ~= 0, continue; end
        n = n+1;  stack = idx;  L(idx) = n;
        while ~isempty(stack)
            p = stack(end);  stack(end) = [];
            [r,c] = ind2sub([H W],p);
            for dr = -1:1
                for dc = -1:1
                    rr = r+dr;  cc = c+dc;
                    if rr>=1 && rr<=H && cc>=1 && cc<=W && M(rr,cc) && L(rr,cc)==0
                        L(rr,cc) = n;
                        stack(end+1) = sub2ind([H W],rr,cc); %#ok
                    end
                end
            end
        end
    end
end

function pts = fk3(q, cfg)   % 3D forward kinematics -> [base; shoulder; elbow; wrist]
    yaw = q(1);  a1 = q(2);  a2 = q(3);
    c  = [cos(yaw) sin(yaw)];
    r1 = cfg.L1*cos(a1);         z1 = cfg.H + cfg.L1*sin(a1);
    r2 = r1 + cfg.L2*cos(a1+a2); z2 = z1 + cfg.L2*sin(a1+a2);
    pts = [0 0 0; 0 0 cfg.H; r1*c z1; r2*c z2];
end

function q = ik3(p, cfg)     % 3D inverse kinematics, p = grasp point [x y z], elbow-up
    yaw = atan2(p(2),p(1));
    r   = hypot(p(1),p(2));
    dz  = p(3) + cfg.hg - cfg.H;               % wrist sits hg above the grasp point
    D = (r^2 + dz^2 - cfg.L1^2 - cfg.L2^2)/(2*cfg.L1*cfg.L2);
    if abs(D) > 1, q = []; return; end
    a2 = -acos(D);
    a1 = atan2(dz,r) - atan2(cfg.L2*sin(a2), cfg.L1+cfg.L2*cos(a2));
    q = [yaw a1 a2];
end

function moveArm(G, q0, q1, carry, T)   % time-based smooth 3D motion
    dq = atan2(sin(q1-q0), cos(q1-q0));            % shortest rotation per joint
    w  = get(G.hG(1),'UserData');                  % current jaw opening
    t0 = tic;
    while true
        t = min(toc(t0)/T, 1);
        s = 10*t^3 - 15*t^4 + 6*t^5;               % minimum-jerk profile
        qc  = q0 + s*dq;
        pts = fk3(qc, G.cfg);
        drawArm(G, pts, qc(1), w);
        if carry > 0
            moveObj(G, carry, pts(4,:) - [0 0 G.cfg.hg]);   % ball rides in the gripper
        end
        drawnow;
        if t >= 1, break; end
    end
end

function setGrip(G, q, w1, T)       % animate the fingers from current opening to w1
    w0  = get(G.hG(1),'UserData');
    pts = fk3(q, G.cfg);  t0 = tic;
    while true
        t = min(toc(t0)/T, 1);
        w = w0 + (w1 - w0)*t;
        drawArm(G, pts, q(1), w);
        set(G.hG(1),'UserData',w);
        drawnow;
        if t >= 1, break; end
    end
end

function drawArm(G, pts, yaw, w)    % redraw tubes, joints and gripper
    cfg = G.cfg;
    sh = pts(2,:);  el = pts(3,:);  wr = pts(4,:);
    [X,Y,Z] = tubeXYZ(sh, el, 0.032);  set(G.hUp,'XData',X,'YData',Y,'ZData',Z);
    setSph(G.hEl, G, el, 0.042);
    [X,Y,Z] = tubeXYZ(el, wr, 0.026);  set(G.hFo,'XData',X,'YData',Y,'ZData',Z);
    % ---- parallel two-finger gripper hanging below the wrist ----
    t  = [-sin(yaw) cos(yaw) 0];                   % jaws open across the arm plane
    pc = wr - [0 0 0.02];                          % palm centre
    pa = pc - 0.075*t;  pb = pc + 0.075*t;         % palm bar ends
    [X,Y,Z] = tubeXYZ(pa, pb, 0.014);  set(G.hG(1),'XData',X,'YData',Y,'ZData',Z);
    setSph(G.hG(2), G, pa, 0.014);
    setSph(G.hG(3), G, pb, 0.014);
    fa = pc - w*t;  fb = pc + w*t;                 % finger tops
    fz = wr(3) - cfg.hg - 0.015;                   % fingertip height
    ta = [fa(1) fa(2) fz];  tb = [fb(1) fb(2) fz]; % fingertips
    [X,Y,Z] = tubeXYZ(fa, ta, cfg.fr);  set(G.hG(4),'XData',X,'YData',Y,'ZData',Z);
    [X,Y,Z] = tubeXYZ(fb, tb, cfg.fr);  set(G.hG(5),'XData',X,'YData',Y,'ZData',Z);
    setSph(G.hG(6), G, ta, 0.013);
    setSph(G.hG(7), G, tb, 0.013);
    setSph(G.hG(8), G, wr, 0.030);
end

function moveObj(G, i, p)           % move ball i (and its shadow) to position p
    setSph(G.hObj(i), G, p, G.cfg.rObj);
    set(G.hSh(i),'XData',p(1),'YData',p(2));
end

function setSph(h, G, c, r)         % place a sphere surface at centre c, radius r
    set(h,'XData',c(1)+r*G.sx,'YData',c(2)+r*G.sy,'ZData',c(3)+r*G.sz);
end

function h = mkSph(ax, col)         % create an empty lit sphere surface
    h = surf(ax,zeros(21),zeros(21),zeros(21),'FaceColor',col,'EdgeColor','none', ...
        'FaceLighting','gouraud','AmbientStrength',0.45,'SpecularStrength',0.5);
end

function h = mkTube(ax, col)        % create an empty lit tube surface (2 x 16 grid)
    h = surf(ax,zeros(2,16),zeros(2,16),zeros(2,16),'FaceColor',col,'EdgeColor','none', ...
        'FaceLighting','gouraud','AmbientStrength',0.45,'SpecularStrength',0.5);
end

function [X,Y,Z] = tubeXYZ(p0, p1, r)   % cylinder surface between points p0 and p1
    p0 = p0(:)';  p1 = p1(:)';
    d = p1 - p0;
    if norm(d) < 1e-9, d = [0 0 1]; else, d = d/norm(d); end
    a = [1 0 0];
    if abs(dot(a,d)) > 0.9, a = [0 1 0]; end
    u = cross(d,a);  u = u/norm(u);
    v = cross(d,u);
    th = linspace(0,2*pi,16);
    X = zeros(2,numel(th));  Y = X;  Z = X;
    for i = 1:2
        c = p0 + (i-1)*(p1-p0);
        P = c' + r*(u'*cos(th) + v'*sin(th));
        X(i,:) = P(1,:);  Y(i,:) = P(2,:);  Z(i,:) = P(3,:);
    end
end
