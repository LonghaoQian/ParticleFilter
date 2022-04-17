% sample a index based on weighting vector
% author: Longhao Qian
% input:
% - wk (N ,1) : weighting vector
% - cumsumWk (N, 1): cumulative sum of the weighting vector
% - mode
% output:
% - idx (1 x 1): sample index
function idx = SampleIndex(wk, cumsumWk, mode)
    
    switch mode
        case 'naive search'
            % generate a rand number from [0 to cumsum]
            U = rand * cumsumWk(end);
            idx = NaiveBinarySearch(cumsumWk, U);
        case 'systemmatic search'
            c = wk(1);
            i = 1;
            k = 1;
            U = rand / Ns;
            while U > c
                i = i + 1;
                c = c + wk(i);
            end
            xkResampled(:, k) = xk(:, i);
            k = k + 1;
        otherwise
            error('incorrect sampling mode');
    end
end