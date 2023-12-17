#!/user/bin/python
import time
import socket
import subprocess
from gpiozero import PWMLED

mot_pinA = {"A": 17, "B": 27, "C": 24}
mot_pinB = {"A": 18, "B": 23, "C": 22}
mot_num = {"A": 0, "B": 1, "C": 2}

mot_A = {"A": PWMLED(17), "B": PWMLED(27), "C": PWMLED(24)}
mot_B = {"A": PWMLED(18), "B": PWMLED(23), "C": PWMLED(22)}

def main():
    return

def start_server():
    host = 'kiwibot.local'
    port = 12345
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"Server listening on {host}:{port}")
    client_socket, client_address = server_socket.accept()

    while True:
        data = client_socket.recv(1024).decode('utf-8')
        output = parse_message(data)
        if output == 2:
            print("Connection closed.  Awaiting new connection.")
            client_socket.shutdown(1)
            client_socket.close()
            client_socket, client_address = server_socket.accept()
        if output == 3:
            client_socket.shutdown(1)
            client_socket.close()
            print("Connection closed.")
            break
    print("Shutting down sockets")
    server_socket.shutdown(1)
    server_socket.close()
    exit()
# MOVE {} {} {}
def parse_message(message):
    print(message)
    if message[0:4] == "MOVE":
        motors = message[5:].split(" ")
        move_motor("A", int(motors[0]))
        move_motor("B", int(motors[1]))
        move_motor("C", int(motors[2]))
        # print(mot_A["A"].value)
        # print(mot_B["A"].value)
        return 1
    if message[0:4] == "QUIT":
        return 2
    if message[0:4] == "SHUT":
        return 3
    return 0

# IN A | IN B | Result
#   0  |  0   | Brake
#   1  |  0   | Clockwise
#   0  |  1   | Counter-clockwise
#   1  |  1   | Coast
def move_motor(motor, power):
    # Power is PWM duty cycle
    # 0 is off, 100 is max
    mot_A[motor].on()
    mot_B[motor].on()
    if power == 0:
        mot_A[motor].value = 0
        mot_B[motor].value = 0
    elif power > 0:
        mot_A[motor].value = min(power / 100.0, 1.0)
        mot_B[motor].value = 0
    else:
        power = -power
        mot_A[motor].value = 0
        mot_B[motor].value = min(power / 100.0, 1.0)

if __name__ == "__main__":
    start_server()
