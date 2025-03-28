import struct
import sys

# コマンドライン引数からファイル名を取得
if len(sys.argv) != 2:
    print("使用法: python script.py <ファイル名>")
    sys.exit(1)

file_path = sys.argv[1]

try:
    # バイナリファイルを読み込む
    with open(file_path, "rb") as f:
        while True:
            # バイナリデータを読み込む
            data = f.read(4 + 4 * 7)  # 4バイト (uint32_t) + 4バイト * 7 (float)
            
            # ファイルの終端に達した場合はループを終了
            if len(data) == 0:
                break

            # 必要なデータが読み込まれているか確認
            if len(data) != 4 + 4 * 7:
                raise ValueError("ファイルのデータ長が予想と一致しません")

            # バイナリデータを指定された型にアンパック
            unpacked_data = struct.unpack('I 7f', data)
            
            timestamp = unpacked_data[0]
            temp = unpacked_data[1]
            gx, gy, gz = unpacked_data[2:5]
            ax, ay, az = unpacked_data[5:8]

            # timestampを19200000.0で割る
            timestamp = timestamp / 19200000.0

            # printf 形式で出力
            print(f"{timestamp:4.6f},{temp:4.2f},{gx:4.2f},{gy:4.2f},{gz:4.2f},{ax:4.2f},{ay:4.2f},{az:4.2f}")
        
except FileNotFoundError:
    print(f"エラー: ファイル '{file_path}' が見つかりません。")
except ValueError as e:
    print(f"エラー: {e}")
