import serial, json, time

# Adjust COM port and baudrate
ser = serial.Serial("COM10", 115200, timeout=1)

while True:
    line = ser.readline().decode(errors="ignore").strip()
    if not line:
        continue
    try:
        msg = json.loads(line)
    except Exception as e:
        print("Bad line:", line, e)
        continue

    if msg.get("t") == "req":
        req_id = msg["id"]
        mac = msg["mac"]
        uid = msg["uid"]

        print("Got request:", msg)

        # Dummy DB check
        status = 1 if uid.startswith("A") else 0

        resp = {
            "t": "resp",
            "id": req_id,
            "mac": mac,
            "status": status,
            "ts": int(time.time()),
            "event": 1,
            "ticket": "T123",
            "name": "Guest"
        }

        ser.write((json.dumps(resp) + "\n").encode())
        print("Sent response:", resp)
