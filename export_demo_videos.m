function export_demo_videos(scn, R, algos, cfg, scenarioId)
%EXPORT_DEMO_VIDEOS Xuat video MP4 rieng cho tung thuat toan trong mot kich ban.
%   Chi nhan ket qua lan chay minh hoa (Phan A), khong ghi video cac lan thong ke.

if nargin < 5 || isempty(scenarioId)
    scenarioId = 'Scenario';
end
outDir = fullfile('videos', scenarioId);
if ~exist(outDir,'dir'), mkdir(outDir); end

for a = 1:numel(algos)
    fileName = sprintf('%s_%s.mp4',scenarioId,upper(algos{a}));
    outputFile = fullfile(outDir,fileName);
    animate_run(scn,R{a},cfg,outputFile);
end
fprintf('\nDa xuat %d video MP4 trong thu muc: %s\n\n',numel(algos),outDir);
end
