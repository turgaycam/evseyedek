import socket
import urllib.request
import json
import concurrent.futures

def check_port80(ip):
    s = socket.socket()
    s.settimeout(0.4)
    try:
        s.connect((ip, 80))
        return ip
    except Exception:
        return None
    finally:
        s.close()

def get_status(ip):
    try:
        with urllib.request.urlopen(f"http://{ip}/status", timeout=3) as r:
            data = json.loads(r.read().decode())
            mac = data.get("mac", "?")
            ver = data.get("otaCur", "?")
            name = data.get("stationName", "?")
            return f"{ip} | {mac} | FW={ver} | {name}"
    except Exception as e:
        return f"{ip} | HATA: {e}"

ips = [f"192.168.1.{i}" for i in range(1, 255)]
with concurrent.futures.ThreadPoolExecutor(max_workers=60) as ex:
    alive = [r for r in ex.map(check_port80, ips) if r]

print(f"HTTP acik cihazlar: {len(alive)}")
for ip in alive:
    print(get_status(ip))