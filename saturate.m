function [y] = saturate(x,min_x,max_x)
if x < min_x
    y = min_x;
elseif x>max_x
    y = max_x;
else
    y = x;
end

end