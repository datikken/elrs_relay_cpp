#!/usr/bin/env python3
# ELRS Relay Server v2 - pair routing (pairId + role)
# Packet: [PAIR_ID(4)] [ROLE(1)] [CRSF data]
# ROLE: 0x00 = pilot, 0x01 = drone
import socket, time

PORT = 14550
BUFFER = 2096
CLIENT_TIMEOUT = 60

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))
    print("ELRS Relay Server v2 on UDP {} (pair routing)".format(PORT))

    pairs = {}

    while True:
        try:
            data, addr = sock.recvfrom(BUFFER)
        except KeyboardInterrupt:
            break

        if len(data) < 5:
            continue

        pair_id = data[:4]
        role = data[4]
        payload = data[5:]

        now = time.time()

        if pair_id not in pairs:
            pairs[pair_id] = {"pilot": None, "drone": None, "last_seen": now}
            print("New pair: {}".format(pair_id.hex()))

        p = pairs[pair_id]
        p["last_seen"] = now
        role_name = "pilot" if role == 0 else "drone"
        if p[role_name] != addr:
            p[role_name] = addr
            print("Pair {} {} -> {}:{}".format(
                pair_id.hex(), role_name, addr[0], addr[1]))

        peer = p["drone"] if role == 0 else p["pilot"]
        if peer:
            sock.sendto(payload, peer)

        dead = [pid for pid, info in pairs.items()
                if now - info["last_seen"] > CLIENT_TIMEOUT]
        for pid in dead:
            print("Pair expired: {}".format(pid.hex()))
            del pairs[pid]

if __name__ == "__main__":
    main()
