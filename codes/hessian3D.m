function [H] = hessian3D(DoG, h, w, DoG_prev, DoG_next)
    % I take width as x axis and height as y axis
    dxx = (DoG(h, w+1) + DoG(h, w-1) - 2*DoG(h, w)) / 1;
    dyy = (DoG(h+1, w) + DoG(h-1, w) - 2*DoG(h, w)) / 1;
    dss = (DoG_next(h, w) + DoG_prev(h, w) - 2*DoG(h, w)) / 1;

    dxy = (DoG(h+1,w+1) + DoG(h-1,w-1) - DoG(h+1,w-1) - DoG(h-1,w+1))/4;
    dxs = (DoG_next(h,w+1) + DoG_prev(h,w-1) - DoG_next(h,w-1) - DoG_prev(h,w+1))/4;
    dys = (DoG_next(h+1,w) + DoG_prev(h-1,w) - DoG_next(h-1,w) - DoG_prev(h+1,w))/4;

    H = [dxx,dxy,dxs; dxy,dyy,dys; dxs,dys,dss];

end
