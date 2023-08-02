import socket
import json
import numpy as np
import time
HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 65430  # The port used by the server

tic = time.time()
angles = np.zeros(20)
angles[10] = np.deg2rad(90)
while time.time() - tic < 300.:
    if (time.time() - tic) % 0.5 == 0:

        position = np.array([0,0,0.35])
        orientation = np.array([ 0.7071068, 0, 0, 0.7071068 ])
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            # angles = np.roll(angles, 1)
            # print(np.where(angles == np.deg2rad(90)))
            data = dict()
            data["angles"] = angles.tolist()
            data["position"] = position.tolist()
            data["orientation"] = orientation.tolist()
            data = json.dumps(data, ensure_ascii=False).encode('utf8')
            s.sendall(data)
            payload = s.recv(4096)
            payload = payload.decode("utf-8").rstrip("\x00")
            print("Received {} from server".format(payload))