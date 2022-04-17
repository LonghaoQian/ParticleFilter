% Resampling function using systematic sampling
% Author: Longhao Qian
% Resampling function
% input: 
% - xk (3, N): particle states
% - wk (1, N): weighting vector
% - resampling_strategy (string): resampling choice
% output:
% - xkResampled (3, N): resampled particles
% - wk (1, N): resampled weights
% - idex (1, N): resampled index
function [xkResampled, wk, idx] = ResampleParticle(xk, wk, resampling_strategy)
    %% video reference
    % https://www.youtube.com/watch?v=MsYlueVDLI0&t=1640s
    % https://www.youtube.com/watch?v=tvNPidFMY20
    % https://www.youtube.com/watch?v=wNQVo6uOgYA
    Ns = length(wk);
    xkResampled = zeros(3, Ns);
    r = rand / Ns;
    c = wk(1);
    i = 1;
    k = 1;
    for j = 1 : Ns
        U = r + (j - 1) / Ns;
        while U > c
            i = i + 1;
            c = c + wk(i);
        end
        xkResampled(:, k) = xk(:, i);
        k = k + 1;
    end
%     % wk = wk./sum(wk); % normalize weight vector (already done)
% 
%     switch resampling_strategy
%        case 'multinomial_resampling'
%           with_replacement = true;
%           idx = randsample(1:Ns, Ns, with_replacement, wk);
%     %{
%           THIS IS EQUIVALENT TO:
%           edges = min([0 cumsum(wk)'],1); % protect against accumulated round-off
%           edges(end) = 1;                 % get the upper edge exact
%           % this works like the inverse of the empirical distribution and returns
%           % the interval where the sample is to be found
%           [~, idx] = histc(sort(rand(Ns,1)), edges);
%     %}
%        case 'systematic_resampling'
%           % this is performing latin hypercube sampling on wk
%           edges = min([0 cumsum(wk)'],1); % protect against accumulated round-off
%           edges(end) = 1;                 % get the upper edge exact
%           u1 = rand/Ns;
%           i = 1;
%           for j = 1 : Ns
%               
%           end
% 
%           % this works like the inverse of the empirical distribution and returns
%           % the interval where the sample is to be found
%           % [~, idx] = histc(u1:1/Ns:1, edges);
%           % [~, ~, idx] = histcounts(u1:1/Ns:1, edges);
%        otherwise
%           error('Resampling strategy not implemented')
%     end
    % xk = xk(:,idx);                    % extract new particles
    % now all particles have the same weight
    wk = repmat(1/Ns, 1, Ns);          
return