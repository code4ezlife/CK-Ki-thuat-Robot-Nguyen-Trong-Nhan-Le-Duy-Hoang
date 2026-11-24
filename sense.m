function [z, ids] = sense(humans, pos, cfg, t, eta)
%SENSE  Số đo vị trí nhiễu (2.5) của các vật cản động trong tầm quét 8 m.
%   eta: nH x 2 số ngẫu nhiên N(0,1) cho bước này (sinh sẵn trong sim_main).
if nargin < 5, eta = randn(numel(humans), 2); end
z = zeros(0,2);  ids = zeros(0,1);
for j = 1:numel(humans)
    h = humans(j);
    if t < h.t_start, continue; end
    if norm(h.p - pos) <= cfg.sense_range
        z(end+1,:)   = h.p + cfg.sigma_z*eta(j,:); %#ok<AGROW>
        ids(end+1,1) = j; %#ok<AGROW>
    end
end
end
