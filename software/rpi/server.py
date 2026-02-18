from comm import RobotServer

server = RobotServer()
server.start()

while True:
    msg = server.recv()
    print("RECV:", msg)
    server.send("ACK:" + msg)