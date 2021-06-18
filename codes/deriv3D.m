function [d] = deriv3D(DoG, h, w, DoG_prev, DoG_next)
    % I take width as x axis and height as y axis
    dx = (DoG(h, w+1) - DoG(h, w-1)) / 2;
    dy = (DoG(h+1, w) - DoG(h-1, w)) / 2;
    ds = (DoG_prev(h, w) - DoG_next(h, w)) / 2;
    d = [dx; dy; ds];
    
end
