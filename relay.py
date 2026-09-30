#!/usr/bin/env python3
import socket, time

PORT = 14550
BUFFER = 2048
CLIENT_TIMEOUT = 30

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))
    print("ELRS Relay Server on UDP {}".format(PORT))
    clients = {}
    while True:
        try:
            data, addr = sock.recvfrom(BUFFER)
        except KeyboardInterrupt:
            break
        now = time.time()
        if addr not in clients:
            print("New client: {}".format(addr))
        clients[addr] = {"last_seen": now}
        dead = [a for a, info in clients.items()
                if now - info["last_seen"] > CLIENT_TIMEOUT]
        for a in dead:
            del clients[a]
        for c in clients:
            if c != addr:
                try: sock.sendto(data, c)
                except OSError: pass

if __name__ == "__main__":
    main()
