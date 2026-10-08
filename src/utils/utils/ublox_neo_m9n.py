import serial
import pynmea2

ser = serial.Serial("/dev/ttyACM0", 38400, timeout=1)

while True:
    line = ser.readline().decode(errors="ignore").strip()
    if line.startswith("$G") and "GSV" in line:
        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            continue
        # Each sentence can report up to 4 satellites
        sats = []
        for i in range(4):
            prn = getattr(msg, f"sv_prn_num_{i+1}", None)
            snr = getattr(msg, f"ss_{i+1}", None)
            if prn is not None:
                sats.append((prn, snr))
        if sats:
            print(f"{msg.talker}: {sats}")