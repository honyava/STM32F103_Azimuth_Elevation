function kama_send(port, coords)
    arguments        
        port
        coords (1, 3) double
    end   

    int_az = uint32(coords(1) * 16384/180);
    int_el = uint32(abs(coords(2) * 16384/180));
    el_is_neg = coords(2) < 0;
    int_r = uint32(coords(3));

    buf = uint8(zeros(1, 26));
    buf(1) = 0xEB;

    buf(12) = uint8(bitshift(bitand(int_az, 0b0111000000000000u32), -12));
    buf(13) = uint8(bitshift(bitand(int_az, 0b0000111111100000u32), -5));
    buf(14) = uint8(bitshift(bitand(int_az, 0b0000000000011111u32), 2));

    buf(15) = uint8(bitshift(bitand(int_r, 0b011000000000000000000000u32), -21));
    buf(16) = uint8(bitshift(bitand(int_r, 0b000111111100000000000000u32), -14));
    buf(17) = uint8(bitshift(bitand(int_r, 0b000000000011111110000000u32), -7));
    buf(18) = uint8(bitand(int_r, 0b000000000000000001111111u32));

    buf(19) = uint8(bitshift(bitand(int_el, 0b0111000000000000u32), -12));
    buf(20) = uint8(bitshift(bitand(int_el, 0b0000111111100000u32), -5));
    buf(21) = uint8(bitshift(bitand(int_el, 0b0000000000011111u32), 2));
    if el_is_neg
        buf(19) = bitor(buf(19), 0x40);
    end

    buf(25) = 0x9C;

    crc = sum(buf);
    crc = mod(crc, 256) + bitshift(crc, -8);
    crc = mod(crc, 256) + bitshift(crc, -8);
    crc = mod(crc, 256) + bitshift(crc, -8);

    buf(26) = uint8(crc);

    port.write(buf, 'uint8');
    fprintf("Send from Kama:");
    fprintf(" 0x%02X", buf);
    fprintf("\n");

end