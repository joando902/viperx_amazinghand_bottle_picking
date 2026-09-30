
import serial
import time

PORT = '/dev/ttyACM0'
# Paxini GEN3 高速板通常為 921600，若無數據可嘗試 115200 或 460800
BAUDRATE = 921600

def test_serial():
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1.0)
        print(f"[*] 成功開啟序列埠 {PORT} @ {BAUDRATE}")
        print("[*] 開始監聽原始 Byte 串流 (按 Ctrl+C 結束)...")

        start_time = time.time()
        byte_count = 0

        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                byte_count += len(data)
                
                # 印出收到的 Hex 格式數據前 32 個 bytes
                hex_str = ' '.join(f'{b:02X}' for b in data[:32])
                print(f"[RECV {len(data):3d} B] {hex_str}")
                
                time.sleep(0.05)
            else:
                # 若 3 秒內都沒收到任何 byte 則提示檢查鮑率
                if time.time() - start_time > 3 and byte_count == 0:
                    print("[!] 尚未收到任何數據，可能原因：鮑率不符或感測器未供電/啟用。")
                    start_time = time.time()
                time.sleep(0.01)

    except serial.SerialException as e:
        print(f"[X] 序列埠錯誤: {e}")
    except KeyboardInterrupt:
        print("\n[*] 測試結束，關閉序列埠。")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    test_serial()
