disp('Start Init');

dev_ports = struct(...
    'pup', find_pup_port(), ...
    'pmes', find_pmes_port(), ...
    'kama', find_kama_port() ...
);

% dev_ports = struct(...
%     'pup', find_pup_port(), ...
%     'pmes', [], ...
%     'kama', find_kama_port() ...
% );

% dev_ports = struct(...
%     'pup', find_pup_port(), ...
%     'pmes', [], ...
%     'kama', [] ...
% );

disp('Complete Init');

%% function
function pup_port = find_pup_port()
    disp("Поиск ПУП...");
    baudrate = 115200;
    ports = serialportlist("available");

    for port = ports
        fprintf("Попытка подключения к порту %s\n", port);
        pup_port = serialport(port, baudrate, 'Timeout', 0.5, 'StopBits',2);
        pup_port.flush();

        w = warning('off', 'all');
        data = pup_port.read(6, "uint8");
        warning(w);

        if ~isempty(data)
            return;
        end

    end

    disp('Плата ПУП не найдена');
    pup_port = [];

end

function pmes_port = find_pmes_port()
    disp("Поиск платы измерений...");
    baudrate = 115200;
    ports = serialportlist("available");

    for port = ports
        fprintf("Попытка подключения к порту %s\n", port);
        pmes_port = serialport(port, baudrate, Timeout = 5);
        pmes_port.flush();

        w = warning('off', 'all');
        pmes_port.write("TEST", "uint8");
        data = pmes_port.read(8, "single");
        warning(w);

        if ~isempty(data)
            return;
        end
    end
    disp('Плата измерений не найдена');
    pmes_port = [];
end

function dev_port = find_kama_port()  
    disp("Поиск порта для Камы..."); 
    baudrate = 115200;
    ports = serialportlist("available");    
    ports = "COM4";
    for port = ports
        fprintf("Попытка подключения к порту %s\n", port);
        dev_port = serialport(port, baudrate, Timeout = 5);
        dev_port.flush();

        w = warning('off', 'all');
        data = 5;
        warning(w);

        if ~isempty(data)
            return;
        end
    end
    disp("Порта для Камы не найден..."); 
    dev_port = [];
end

