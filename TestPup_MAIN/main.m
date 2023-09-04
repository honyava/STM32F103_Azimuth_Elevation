clear
close all
kama_cnt = 0;

AZIMUTH_TEST = 0:2:359;
ELEVATION_TEST = [2.89];
R_TEST = [1024];

% название платы для теста "az" или "el"
pup_name = 'az';
run("init");

pup_port = dev_ports.pup;
pmes_port = dev_ports.pmes;
kama_port = dev_ports.kama;

% kama_send(dev_ports.kama, [50.0 12.0 54545]);

% Задание режима ПУП
% pup_write(pup_name, pup_port, 0x1, 1); %% 1-normal, 2-kama
pup_write(pup_name, pup_port, 0x1, 2); %% 1-normal, 2-kama
pause(0.5);
% input("Нажмите Enter для продолжения");
% pup_send_cor_kama_first_azat(pup_name, pup_port, 0, -1.5);
pause(0.5);
% test_pup_send_deg(pup_name, pup_port, pmes_port);

%%%%%%%%%TEST
% test_kama_send_deg(kama_port, pmes_port, 0.01, AZIMUTH_TEST, ELEVATION_TEST, R_TEST);
%%%%%%%%%TEST

%%%%%%%%%%%%%%%%Test function
% pup_get_ver(pup_name, pup_port);
pause(0.2);
% pup_send_cor_kama_first_azat(pup_name, pup_port, 1, 0);
pause(0.2);
% pup_send_cor_kama_first_r(pup_name, pup_port, 200);
% pause(0.2);
% pup_send_kama_pos(pup_name, pup_port, 5, 5, 5);
% pause(0.2);

%%%%%%%%%%%%%%%%Test function

% Задание коррекции ошибки
% pup_send_kama_pos(pup_name, pup_port, 30, 20, 10);
pause(0.1);
pup_send_deg(pup_name, pup_port, 0);
pup_send_cor_first(pup_name, pup_port, 3)
kama_send(kama_port, [0, ELEVATION_TEST, R_TEST]);
% return
input("Нажмите Enter для начала теста")

f = figure;
p = plot(0, 0, 'o', 'MarkerFaceColor', 'red');
al = animatedline('Color', [0 .7 .7]);
txt = text(0, 0, "");

% deg_test = (0:2:360)';
deg_test = AZIMUTH_TEST;
k = 2;

deg_ref = repelem(deg_test, k);
deg_mes = deg_ref * 0;

deg_delta = deg_ref * 0;
Asin = deg_ref * 0;
Acos = deg_ref * 0;
Azap = deg_ref * 0;
dsin = deg_ref * 0;
dcos = deg_ref * 0;
dzap = deg_ref * 0;
dAz = deg_ref * 0;
dEl = deg_ref * 0;

xticks(1:k:length(deg_ref));
xticklabels(deg_test);

pup_send_deg(pup_name, pup_port, deg_test(1));
pause(0.2);
pup_send_deg(pup_name, pup_port, deg_test(1));
pause(0.2);
while true
for i = 1:length(deg_ref)
    if mod(i, k) == 1
        fprintf("\n"); 
%          pup_send_deg(pup_name, pup_port, deg_ref(i));    
%         kama_send(kama_port, [deg_ref(i), 50, 3000]);    
          kama_send(kama_port, [deg_ref(i), ELEVATION_TEST, R_TEST]);
          pause(0.2);
         kama_cnt = kama_cnt + 1;
        fprintf("Ref degree %4.1f\n", deg_ref(i)); 
        fprintf(" Asin\t Acos\t Azap\t dsin\t dcos\t dzap\t dAz\t dEl\n"); 
    end

    pmes_port.write("TEST", "uint8");
    %data = pmes_port.pmes.read(8, "single");
    data = pmes_port.read(8, "single");
    pause(0.1);
    Asin(i) = data(1);
    Acos(i) = data(2);
    Azap(i) = data(3);
    dsin(i) = data(4);
    dcos(i) = data(5);
    dzap(i) = data(6);
    dAz(i) = data(7);
    dEl(i) = data(8);
    % disp(data);
     deg_mes(i) = data(7); %for azimuth
 %   deg_mes(i) = data(8); %for elevation
    fprintf("%6.1f |%6.1f |%6.1f |%6.1f |%6.1f |%6.1f |%6.1f |%6.1f\n", data);   
    [az_out, el_out, r_out] = ParalaxCalcRef(deg_ref(i), 2.89, 1024);
    deg_delta(i) = az_out - deg_mes(i);
%    deg_delta(i) = deg_ref(i) - deg_mes(i);

    if deg_delta(i) > 180
        deg_delta(i) = deg_delta(i) - 360;
    elseif deg_delta(i) < -180
        deg_delta(i) = deg_delta(i) + 360;
    end

    x = i;
    y = deg_delta(i);

    if isvalid(f)
        addpoints(al, x, y);
        p.XData = x;
        p.YData = y;
        txt.Position = [x, y];
        txt.String = ['\leftarrow' num2str(y)];
        drawnow limitrate
    else
        break;
    end
end
end

outdata = [deg_ref, deg_mes, deg_delta, ];
save('data.mat', 'outdata');

input("Нажмите Enter для продолжения")
% pup_send_cor(pup_name, pup_port, 0.5);
pup_write(pup_name, pup_port, 0x1, 1);
delete(pmes_port);
delete(pup_port);
delete(kama_port);

disp("Press Ctrl+C to exit ;)");

while true
end

%% function
function pup_get_ver(name, port)
    arguments
        name
        port        
    end 
    pup_write(name, port, 0xF, 0)
end

function pup_send_deg(name, port, deg)
    arguments
        name
        port        
        deg (1,1) double
    end
    cmd = typecast(int32(deg * 10), "uint32");
    pup_write(name, port, 0x2, cmd)
end

function pup_send_cor_kama_first_azat(name, port, cor_az, cor_el)
    arguments
        name
        port        
        cor_az (1,1) double
        cor_el (1,1) double
    end
    cmdh = typecast(int32(cor_az * 2^15 / 180), "uint32");
    cmdl = typecast(int32(cor_el * 2^15 / 180), "uint32");
    cmd = uint32(bitshift(cmdh, 16) + bitand(cmdl, 0xFFFFu32));
    pup_write(name, port, 0x9, cmd)
end

function pup_send_cor_kama_first_r(name, port, cor_r)
    arguments
        name
        port        
        cor_r (1,1) double
    end
    cmd = typecast(int32(cor_r), "uint32");
    cmd = uint32(bitand(cmd, 0xFFFFu32));
    pup_write(name, port, 0xA, cmd)
end

function pup_send_kama_pos(name, port, x, y, z)
    arguments
        name
        port        
        x (1,1) double
        y (1,1) double
        z (1,1) double
    end
    cmdx = typecast(int32(x), "uint32");
    cmdy = typecast(int32(y), "uint32");
    cmdz = typecast(int32(z), "uint32");
    cmd = uint32(...
        bitshift(bitand(cmdx, 0xFFu32), 16) + ...
        bitshift(bitand(cmdy, 0xFFu32), 8) + ...
        bitand(cmdz, 0xFFu32)  ...
    );
    pup_write(name, port, 0xB, cmd)
end

function pup_send_cor(name, port, correction)
    arguments
        name
        port        
        correction (1,1) double
    end
    cmd = typecast(int32(correction * 10), "uint32");
    cmd = uint32(bitshift(cmd, 16));
    pup_write(name, port, 0x6, cmd)
end

function pup_send_cor_first(name, port, correction)
    arguments
        name
        port        
        correction (1,1) double
    end
    cmd = typecast(int32(correction * 10), "uint32");
    cmd = uint32(bitshift(cmd, 16));
    pup_write(name, port, 0x5, cmd)
end

function pup_write(name, port, id, cmd)
    arguments
        name
        port
        id (1,1) uint8
        cmd (1,1) uint32
    end

    id = bitand(id, 0xF);
    if name == 'az'
        id = bitor(id, 0x10);
    elseif name == 'el'
        id = bitor(id, 0x20);
    end

    id = uint8(id);
    cmd = uint32(cmd);
    message = uint8(zeros(6, 1));
    message(1) = id;
    message(2:5) = fliplr(typecast(cmd, 'uint8'));
    message(6) = mod(sum(message), 256);
    port.write(message, 'uint8');
    fprintf("Send to Pup:");
    fprintf(" 0x%02X", message);
    fprintf("\n");
end




