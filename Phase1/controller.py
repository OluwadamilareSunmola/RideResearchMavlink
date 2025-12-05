import serial
import time

class DroneController:
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.2)

    def send(self, payload):
        self.ser.write(payload.encode())

    def receive(self):
        data = self.ser.readline().decode().strip()
        return data

    def wait_for_ack(self):
        while True:
            msg = self.receive()
            if msg in ("ACK", "ERR"):
                return msg
            time.sleep(0.05)

    def wait_for_done(self):
        while True:
            msg = self.receive()
            if msg in ("done", "idle"):
                return msg
            time.sleep(0.05)

    def command(self, letter):
        self.send(letter)
        ack = self.wait_for_ack()
        print("ACK status:", ack)

        if ack == "ERR":
            return

        done = self.wait_for_done()
        print("Execution status:", done)

    # Drone commands
    def takeoff(self): self.command("A")
    def up(self):      self.command("B")
    def down(self):    self.command("C")
    def square(self):  self.command("D")
    def land(self):    self.command("E")

if __name__ == "__main__":
    ctrl = DroneController()

    while True:
        print("\nCommands:")
        print("1 Takeoff")
        print("2 Up")
        print("3 Down")
        print("4 Square")
        print("5 Land")
        choice = input("Command: ").strip()

        mapping = {
            "1": ctrl.takeoff,
            "2": ctrl.up,
            "3": ctrl.down,
            "4": ctrl.square,
            "5": ctrl.land
        }

        if choice in mapping:
            mapping[choice]()
        else:
            print("Invalid choice")
