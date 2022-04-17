% naive binary search
% num is given in ascending order
function idex = NaiveBinarySearch(num, target)
    idex = 1;
    if target < num(1)
        return;
    end
    if target > num(end)
        idex = length(num);
        return;
    end
    upper = length(num);
    lower = 1;
    while upper - lower > 1
        mid = lower + floor((upper - lower) / 2);
        if (num(mid) < target)
            lower = mid;
        else
            upper = mid;
        end
    end
    idex = upper;
end