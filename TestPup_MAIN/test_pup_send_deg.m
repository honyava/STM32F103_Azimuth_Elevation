function test_pup_send_deg(pup_name, pup_port, pmes_port)
    fprintf("Test Pup\n");
    angles = sort(randi([0, 360], 1, 6));
    for i = 1:numel(angles)
        pup_send_deg(pup_name, pup_port, angles(i));
        pause(0.1);
        pmes_port.write("TEST", "uint8");
    % проверить, что угол, который мы отправили, совпадает с углом, который мы прочитали
        data = pmes_port.read(8, "single");
        pause(0.1);
        deg_mes = data(7);
        assert(round(abs(angles(i)) - round(deg_mes)) <= 1);
        fprintf("PUP Az %6.1f: Ok\n", deg_mes);
    end
end