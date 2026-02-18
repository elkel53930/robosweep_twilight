import socket
import threading
import time
import queue
from zeroconf import Zeroconf, ServiceInfo, ServiceBrowser

SERVICE_TYPE = "_robot._tcp.local."
PORT = 50000


# ===============================
# Utility
# ===============================

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip


# ===============================
# Robot Server
# ===============================

class RobotServer:
    def __init__(self):
        self.sock = None
        self.conn = None
        self.recv_queue = queue.Queue()
        self.running = True

    def start(self):
        ip = get_local_ip()
        self.zeroconf = Zeroconf()

        info = ServiceInfo(
            SERVICE_TYPE,
            "RobotServer._robot._tcp.local.",
            addresses=[socket.inet_aton(ip)],
            port=PORT,
            properties={},
        )
        self.zeroconf.register_service(info)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", PORT))
        self.sock.listen(1)

        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self):
        while self.running:
            conn, addr = self.sock.accept()
            self.conn = conn
            threading.Thread(target=self._recv_loop, daemon=True).start()

    def _recv_loop(self):
        buffer = b""
        while self.running and self.conn:
            try:
                data = self.conn.recv(1024)
                if not data:
                    self.conn = None
                    break
                buffer += data
                while b"\n" in buffer:
                    msg, buffer = buffer.split(b"\n", 1)
                    self.recv_queue.put(msg.decode())
            except:
                self.conn = None
                break

    def send(self, msg: str):
        if self.conn:
            self.conn.sendall((msg + "\n").encode())

    def recv(self, timeout=None):
        return self.recv_queue.get(timeout=timeout)


# ===============================
# Robot Client
# ===============================

class RobotClient:
    def __init__(self):
        self.sock = None
        self.server_ip = None
        self.recv_queue = queue.Queue()
        self.running = True

    def start(self):
        threading.Thread(target=self._connect_loop, daemon=True).start()

    # ---- mDNS discovery ----
    class _Listener:
        def __init__(self, outer):
            self.outer = outer

        def add_service(self, zeroconf, type, name):
            info = zeroconf.get_service_info(type, name)
            if info:
                self.outer.server_ip = socket.inet_ntoa(info.addresses[0])

        def update_service(self, zeroconf, type, name):
            pass

    def _discover(self):
        zeroconf = Zeroconf()
        listener = self._Listener(self)
        ServiceBrowser(zeroconf, SERVICE_TYPE, listener)

        while self.server_ip is None and self.running:
            time.sleep(0.2)

        zeroconf.close()

    # ---- connect loop ----
    def _connect_loop(self):
        while self.running:
            try:
                if self.server_ip is None:
                    self._discover()

                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((self.server_ip, PORT))

                threading.Thread(target=self._recv_loop, daemon=True).start()
                return

            except:
                time.sleep(1)  # 1秒リトライ

    def _recv_loop(self):
        buffer = b""
        while self.running:
            try:
                data = self.sock.recv(1024)
                if not data:
                    raise ConnectionError

                buffer += data
                while b"\n" in buffer:
                    msg, buffer = buffer.split(b"\n", 1)
                    self.recv_queue.put(msg.decode())

            except:
                self.server_ip = None
                time.sleep(1)
                self._connect_loop()
                break

    def send(self, msg: str):
        if self.sock:
            self.sock.sendall((msg + "\n").encode())

    def recv(self, timeout=None):
        return self.recv_queue.get(timeout=timeout)
