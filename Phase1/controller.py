import serial
import time

class DroneController:
    def __init__(self, port="/dev/ttyUSB0", baud=9600):
        self.ser = serial.Serial(port, baud, timeout=1)

    def send(self, payload):
        self.ser.write(payload.encode())

    def receive(self):
        data = self.ser.readline().decode().strip()
        return data

    def wait_until_ready(self):
        while True:
            msg = self.receive()
            if msg == "ready" or msg == "idle" or msg == "done":
                break
            time.sleep(0.05)

    def takeoff(self):
        self.send("A")
        self.wait_until_ready()

    def up(self):
        self.send("B")
        self.wait_until_ready()

    def down(self):
        self.send("C")
        self.wait_until_ready()

    def square(self):
        self.send("D")
        self.wait_until_ready()

    def land(self):
        self.send("E")
        self.wait_until_ready()

    def loop(self):
        while True:
            msg = self.receive()
            if msg:
                print("STM:", msg)
            time.sleep(0.05)


if __name__ == "__main__":
    ctrl = DroneController()

    while True:
        print("1 Takeoff")
        print("2 Up")
        print("3 Down")
        print("4 Square")
        print("5 Land")
        choice = input("Command: ").strip()

        if choice == "1":
            ctrl.takeoff()
        elif choice == "2":
            ctrl.up()
        elif choice == "3":
            ctrl.down()
        elif choice == "4":
            ctrl.square()
        elif choice == "5":
            ctrl.land()

        resp = ctrl.receive()
        if resp:
            print("STM:", resp)
