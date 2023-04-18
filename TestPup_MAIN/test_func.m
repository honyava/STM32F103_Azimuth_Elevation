
function test_func(pup_name, pup_port)
    angles = sort(randi([0, 360], 1, 6));
    for i = 1:numel(angles)
        pup_send_deg(pup_name, pup_port, angles(i));
    % проверить, что угол, который мы отправили, совпадает с углом, который мы прочитали
        deg_mes = pmes_port.read(8, "single");
        assert(deg_test == deg_mes);
        fprintf("PUP %6.1f: Ok", deg_mes);
        pause(0.1);
    end
end

function test_kama_send_deg(kama_port)
    angles = sort(randi([0, 360], 1, 6));
    for i = 1:numel(angles)
        kama_send(kama_port, [angles(i), 10, 2000]);
    % проверить, что угол, который мы отправили, совпадает с углом, который мы прочитали
        deg_mes = pmes_port.read(8, "single");
        [az_out, el_out, r_out] = ParalaxCalcRef(angles(i), 10, 2000);
        assert(deg_test == deg_mes);
        fprintf("KAMA %6.1f: Ok", deg_mes);
        pause(0.1);
    end
end