#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
讀取 E220-400T30D（地面站）串口的封包，存成 CSV，並在主控台即時顯示。
以酬載種類分區顯示：GNSS / ADC / Temp
- GNSS: UTC, ALT, LON, LAT
- ADC : VBAT, VSOLAR, V3V3, V5V, V3V3EF, EPS, TS
- Temp: T
未知鍵彙整到單一欄位 PAY（內容為 "K1=V1 K2=V2 ..."）

封包示例：
DATA,<millis>,T=<°C>,UTC=<yyyy-mm-ddThh:mm:ssZ>,ALT=<m>,LON=<°>,LAT=<°>,VBAT=<V>,VSOLAR=<V>,V3V3=<V>,V5V=<V>,V3V3EF=<V>,TS=,EPS=
"""

import argparse, csv, os, sys, time, signal
from datetime import datetime
import serial
from serial.tools import list_ports

# ======== CSV 欄位定義（只留一個 PAY） ========
CSV_FIELDS = [
    "Pkt-Cnt",
    "rx_time", "port", "millis",  # millis 保留在 CSV，主控台不顯示
    "UTC", "Alt", "Lng", "Lat",   # GNSS
    "VBAT", "VSOLAR", "V3V3", "V5V", "V3V3EF", "EPS", "TS",  # ADC
    "T",
    "H",                         # Temp
    "PAY",                        # 未知鍵聚合
    "Ax",
    "Ay",
    "Az",
    "Gx",
    "Gy",
    "Gz",
    "Sat",
    "raw"
]

# 已知欄位對應（DATA 內的 key → CSV 欄位名）
KNOWN_MAPPING = {
    "Pkt-Cnt":"Package",
    "millis": "millis",
    "UTC": "UTC", "Alt": "Alt", "Lng": "Lng", "Lat": "Lat",
    "VBAT": "VBAT", "VSOLAR": "VSOLAR", "V3V3": "V3V3", "V5V": "V5V", "V3V3EF": "V3V3EF", "EPS": "EPS", "TS": "TS",
    "T": "T",
    "H": "H",
    "Ax": "Ax",
    "Ay": "Ay",
    "Az": "Az",
    "Gx": "Gx",
    "Gy": "Gy",
    "Gz": "Gz",
    "Sat": "Sat"
}

# ======== 解析一行 DATA 封包 ========
def parse_line(line: str):
    """
    只處理以 'DATA,' 開頭的封包；回傳 dict（至少包含 'millis' 與 'raw'）
    其餘 key 會原樣帶入（例如 T、UTC、ALT、LON、LAT、VBAT...）
    """
    line = line.strip()
    if not line.startswith("Package="):
        return None

    parts = line.split(",")
    if len(parts) < 2:
        return None

    out = {"raw": line}
    out["UTC"] = parts[1].strip()  # DATA 後第一個欄位是 millis

    for token in parts[2:]:
        token = token.strip()
        if "=" in token:
            k, v = token.split("=", 1)
            k = k.strip(); v = v.strip()
            if k:
                out[k] = v
    return out

# ======== CSV 表頭處理 ========
def ensure_header(csv_path, fieldnames):
    need_header = (not os.path.exists(csv_path)) or os.path.getsize(csv_path) == 0
    f = open(csv_path, "a", newline="", encoding="utf-8")
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    if need_header:
        writer.writeheader()
        f.flush()
    return f, writer

# ======== USB/COM 埠挑選（放在流程最前） ========
def pick_port(port_arg: str):
    """
    若 port_arg != 'AUTO' 則直接用；否則自動挑第一個可用埠。
    """
    if port_arg and port_arg.upper() != "AUTO":
        return port_arg

    ports = list(list_ports.comports())
    if not ports:
        print("[ERR] 找不到任何可用的串口（請接上 E220 並安裝驅動）")
        sys.exit(2)

    preferred = None
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in ["usb", "uart", "serial", "silicon", "cp210", "ftdi", "ch340"]):
            preferred = p
            break
    chosen = preferred or ports[0]

    print("可用串口：")
    for p in ports:
        print(f"  {p.device:12s} {p.description}")
    print(f"[INFO] 已自動選用：{chosen.device} （可用 --port 指定，或 --show-ports 查看）")
    return chosen.device

# ======== 主控台即時顯示（分三大類） ========
def print_live(data_row: dict, pay_text: str):
    """
    主控台輸出分為 GNSS / ADC / Temp 三區。
    只顯示 UTC（不顯示 millis），若無 UTC 則退回 rx_time。
    """
    def nz(k, d=""):
        return data_row.get(k) or d

    show_time = nz("UTC", nz("rx_time"))
    gnss = f"[GNSS] UTC={show_time} ALT={nz('ALT')} LON={nz('LON')} LAT={nz('LAT')}"
    adc  = f"[ADC] VBAT={nz('VBAT')} VSOLAR={nz('VSOLAR')} 3V3={nz('V3V3')} 5V={nz('V5V')} 3V3EF={nz('V3V3EF')} EPS={nz('EPS')} TS={nz('TS')}"
    tmp  = f"[Temp] T={nz('T')}"
    line = f"{gnss} | {adc} | {tmp}"
    if pay_text:
        line += f" | [PAY] {pay_text}"
    print(line)

# ======== 主程式 ========
def main():
    ap = argparse.ArgumentParser(description="E220 下行封包 → CSV 紀錄器（GNSS/ADC/Temp 分區 + 單一 PAY）")
    # 將「port」放第一個；預設 AUTO
    ap.add_argument("--port", default="AUTO",
                    help="序列埠（AUTO=自動挑選；Win: COM6/COM8...；Linux: /dev/ttyUSB0；macOS: /dev/tty.usbserial-xxx）")
    ap.add_argument("--baud", type=int, default=9600, help="鮑率，需與 E220 PC 端一致（預設 9600）")
    ap.add_argument("--outfile", default="telemetry.csv", help="輸出 CSV 檔名（預設 telemetry.csv）")
    ap.add_argument("--show-ports", action="store_true", help="列出本機可用串口後結束")
    args = ap.parse_args()

    if args.show_ports:
        print("可用串口：")
        for p in list_ports.comports():
            print(f"  {p.device:12s} {p.description}")
        return

    # ===== 先處理 USB/COM 埠 =====
    chosen_port = pick_port(args.port)

    # ===== 開啟序列埠與 CSV =====
    try:
        ser = serial.Serial(chosen_port, args.baud, timeout=0.5)
    except Exception as e:
        print(f"[ERR] 無法開啟序列埠 {chosen_port}: {e}")
        print("提示：用 --show-ports 看看實際埠名，或確認驅動與線材。")
        sys.exit(1)

    f, writer = ensure_header(args.outfile, CSV_FIELDS)
    print(f"[OK] 正在監聽 {chosen_port}，寫入 {args.outfile}（Ctrl+C 結束）")

    # ===== 優雅關閉 =====
    def _graceful_exit(signum, frame):
        try:
            ser.close()
        except: pass
        try:
            f.close()
        except: pass
        print("\n[INFO] 已關閉檔案與序列埠。")
        sys.exit(0)

    signal.signal(signal.SIGINT, _graceful_exit)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _graceful_exit)

    # ===== 主迴圈 =====
    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            parsed = parse_line(line)
            if parsed is None:
                # 非 DATA 封包也顯示
                print(f"[RX] {line}")
                continue

            # 形成 CSV row
            data_row = {k: "" for k in CSV_FIELDS}
            data_row["rx_time"] = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
            data_row["port"] = chosen_port
            data_row["raw"] = line

            # 已知欄位填入
            for k_src, k_csv in KNOWN_MAPPING.items():
                if k_src in parsed:
                    data_row[k_csv] = parsed[k_src]

            # 未知鍵彙整到單一 PAY 欄位（"K1=V1 K2=V2 ..."）
            unknown_pairs = []
            for k, v in parsed.items():
                if k in ("raw",) or k in KNOWN_MAPPING:
                    continue
                unknown_pairs.append(f"{k}={v}")
            if unknown_pairs:
                data_row["PAY"] = " ".join(unknown_pairs)

            # 寫入 CSV
            writer.writerow(data_row)
            f.flush()

            # 主控台三分區輸出（只顯示 UTC，不顯示 millis）
            print_live(data_row, data_row.get("PAY", ""))

        except Exception as e:
            print(f"[ERR] {e}")
            time.sleep(0.2)

if __name__ == "__main__":
    main()
