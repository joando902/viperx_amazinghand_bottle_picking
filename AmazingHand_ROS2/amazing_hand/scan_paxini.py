import serial
import time

PORT = '/dev/ttyACM0'
BAUDRATES = [921600, 115200, 460800, 2000000, 1000000, 57600]

# 常見的 Paxini / 串列感測器啟動指令
START_COMMANDS = [
    b'',                          # 純被動監聽
    b'\xAA\x55\x01\x00\x00\xFF',  # 啟動串流指令 1
    b'\xAA\x56\x01\x01',          # 啟動串流指令 2
    b'start\r\n',                 # ASCII 指令
]

print(f"=== 開始掃描 {PORT} ===")

for baud in BAUDRATES:
    print(f"\n[?] 測試鮑率: {baud} ...")
    try:
        ser = serial.Serial(PORT, baud, timeout=0.3)
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        found_data = False
        for cmd in START_COMMANDS:
            if cmd:
                ser.write(cmd)
                time.sleep(0.1)

            start_t = time.time()
            while time.time() - start_t < 0.5:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    hex_str = ' '.join(f'{b:02X}' for b in data[:32])
                    print(f"  [+] 成功收到數據！ (鮑率: {baud}) -> [長度 {len(data)} B] {hex_str}")
                    found_data = True
                    break
                time.sleep(0.02)
            
            if found_data:
                break
        
        ser.close()
        if found_data:
            print(f"\n>>> 找到正確鮑率: {baud} <<<")
            break

    except Exception as e:
        print(f"  [X] 開啟失敗: {e}")

print("\n=== 掃描結束 ===")
