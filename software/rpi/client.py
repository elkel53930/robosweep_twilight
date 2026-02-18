from comm import RobotClient

client = RobotClient()
client.start()

while True:
    msg = input("Send: ")
    client.send(msg)
    print("RECV:", client.recv())
