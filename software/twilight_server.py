import time

from rpi.comm import RobotServer


def main():
    server = RobotServer()
    server.start()

    try:
        while True:
            try:
                client_id, msg = server.recv(timeout=0.2)
            except Exception:
                continue

            print(f"Received from client {client_id}: {msg}")

            target_id = 2 if client_id == 1 else 1
            server.send(target_id, msg)

    except KeyboardInterrupt:
        pass
    finally:
        time.sleep(0.1)


if __name__ == "__main__":
    main()

