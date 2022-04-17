% test the sample policy
% number of trials
N = 2000;
idex_seq = zeros(N, 1);
wk = [0.33, 0.1, 0.2, 0.4, 0.6, 0.99, 2, 7, 0.3, 0.1];
ck = cumsum(wk);
for i = 1 : N
    idex_seq(i) = SampleIndex(wk, ck, 'naive search');
end
% count the number of hits for each index
hk = zeros(size(wk));
for i = 1 : N
    hk(idex_seq(i)) = hk(idex_seq(i)) + 1;
end
hk = hk / N;
wk_norm = wk / ck(end);
%% plot results
close all
figure(1)
hold on
for i = 1 : length(wk)
    plot([i, i], [0 hk(i)], '-o')
    plot([i, i], [0 wk_norm(i)], '-x')
end
grid on