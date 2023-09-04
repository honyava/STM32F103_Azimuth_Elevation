function test_kama_send_deg(kama_port, pmes_port, accuracy, azimuth, elevation, r)
    fprintf("Test KAMA\n");
    angles = (0:accuracy:360)';
    el = (0:accuracy:90)';
    flag = 1;
    for k = 1:length(elevation)
        if (flag == 1)
            for i = 1:numel(el)
                kama_send(kama_port, [0, el(i), r]);
                if(el(i) == elevation(k))
                    flag = 0;
                    pause(1);
                    break;
                end
            end
        else 
            break;
        end
        for j = 1:length(azimuth)
%             pup_send_deg(pup_name, pup_port, 0);
            for i = 1:numel(angles)
                kama_send(kama_port, [angles(i), elevation(k), r]);
                if(angles(i) == azimuth(j))
                    pause(2);
                    pmes_port.write("TEST", "uint8");
                    % проверить, что угол, который мы отправили, совпадает с углом, который мы прочитали
                    data = pmes_port.read(8, "single");
                    pause(1);
                    deg_mes = data(7);
                    [az_out, el_out, r_out] = ParalaxCalcRef(angles(i), elevation(k), r);
                    fprintf("KAMA Az %6.2f\n", deg_mes);
                    assert(round(abs(deg_mes - az_out)) <= 2);
                    fprintf("KAMA Az %6.2f: Ok\n", deg_mes);
                end
            end
        end
        flag = 1;
    end
    fprintf("Test finish\n");
    fprintf("KAMA deg_mes %6.2f: Ok\n", deg_mes);
    fprintf("KAMA el_out  %6.2f: Ok\n", el_out);
    fprintf("KAMA az_out  %6.2f: Ok\n", az_out);
end