import time
from midas_hand_api.tactile.paxini import PaxiniClient, PaxiniConfig

# 根據你實際插上的手指調整（例如如果你只接了 thumb 與 index）
# 可選: ['thumb', 'index', 'middle', 'ring']
CONNECTED_FINGERS = ['thumb', 'index']  

print(f"[*] 嘗試連線至 /dev/ttyACM0 (啟用手指: {CONNECTED_FINGERS})...")

try:
    # 建立配置並指定手指列表
    config = PaxiniConfig(
        port='/dev/ttyACM0',
        fingers=CONNECTED_FINGERS
    )
    client = PaxiniClient(config)
    client.connect()
    
    print("[+] 成功連線！開始進行基準點歸零校正 (請勿觸碰感測器)...")
    client.recalibrate()
    print("[+] 校正完成，開始即時讀取觸覺數據 (按 Ctrl+C 結束)...")

    while True:
        frame = client.read_latest()
        if frame:
            # 印出各指的總垂直力或第一維度平均力
            for finger_name, tactile_data in frame.items():
                # tactile_data 通常為 numpy array (N x 3) [Fx, Fy, Fz]
                print(f"[{finger_name}] shape: {tactile_data.shape}, 均值: {tactile_data.mean(axis=0)}")
        time.sleep(0.05)

except Exception as e:
    print(f"\n[!] 執行發生錯誤: {e}")
