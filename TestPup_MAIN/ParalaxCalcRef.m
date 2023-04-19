%u32 Az град = 180 * 2 ^-14 * az_in
%i32 At град = 180 * 2 ^-14 * el_in
%u32 r м     = r_in

function [az_out, el_out, r_out] = ParalaxCalcRef(az_in, el_in, r_in)%#codegen
    x_base = 29;
    y_base = 17;
    z_base = 11;


    az_in = deg2rad(az_in);
    el_in = deg2rad(el_in);

    [x_in, y_in, z_in] = sph2cart(az_in, el_in, r_in);

    x_out = x_in + x_base;
    y_out = y_in + y_base;
    z_out = z_in + z_base;

    [az_out, el_out, r_out] = cart2sph(x_out, y_out, z_out);
    az_out(az_out < 0) = az_out(az_out < 0) + 2 * pi;

    az_out = rad2deg(az_out);
    el_out = rad2deg(el_out);
end
