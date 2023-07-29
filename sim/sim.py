import numpy as np
import pybullet as pb
from pybullet_utils.bullet_client import BulletClient
import argparse
import socket
import json
import select
import sys
import time
from mano_pybullet.hand_body import HandBody
from mano_pybullet.hand_model import HandModel20
from mano_pybullet.envs.objects import YCBObject
import os
import pybullet_data



HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 65430  # Port to listen on (non-privileged ports are > 1023)


def parse_args():
    parser = argparse.ArgumentParser('GUI debug tool')
    parser.add_argument('--dofs', type=int, default=20,
                        help='Number of degrees of freedom (20 or 45)')

    parser.add_argument('--left-hand', dest='left_hand', action='store_true',
                        help='Show left hand')
    parser.add_argument('--right-hand', dest='left_hand', action='store_false',
                        help='Show right hand')
    parser.set_defaults(left_hand=False)

    parser.add_argument('--visual-shapes', dest='visual', action='store_true',
                        help='Show visual shapes')
    parser.add_argument('--no-visual-shapes', dest='visual', action='store_false',
                        help='Hide visual shapes')
    parser.set_defaults(visual=True)

    parser.add_argument('--self-collisions', dest='self_collisions', action='store_true',
                        help='Enable self collisions')
    parser.add_argument('--no-self-collisions', dest='self_collisions', action='store_false',
                        help='Disable self collisions')
    parser.set_defaults(self_collisions=True)
    return parser

def main():
    args = parse_args()

    """Test GUI application."""
    client = BulletClient(pb.GUI)
    client.setGravity(0, 0, -10)

    client.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=-40.0,
        cameraPitch=-40.0,
        cameraTargetPosition=[0.0, 0.0, 0.0]
    )
    # args.left_hand = True
    hand_model = HandModel20(True)

    flags = sum([
        HandBody.FLAG_ENABLE_COLLISION_SHAPES,
        HandBody.FLAG_ENABLE_VISUAL_SHAPES * True,
        HandBody.FLAG_JOINT_LIMITS,
        HandBody.FLAG_DYNAMICS,
        HandBody.FLAG_USE_SELF_COLLISION * True])

    hand = HandBody(client, hand_model, flags=flags)
    urdfRootPath = pybullet_data.getDataPath()


    client.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.65], useFixedBase=1)


    client.setRealTimeSimulation(True)

    position = np.zeros(3)
    position[-1] = 0.35
    orientation = np.array([ 0.7071068, 0, 0, 0.7071068 ]) 
    angles = np.zeros(20)   
    hand.reset(position, orientation, angles)

    obj = YCBObject("036_wood_block")
    obj.load()
    client.resetBasePositionAndOrientation(obj.body_id, [0,0,0.01], [0.707, 0, 0, 0.707])
    # client.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0., 0, 0.01], baseOrientation=[ 0, 0, 1, 0 ], useFixedBase=1, globalScaling=0.1)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen()
    s.settimeout(600.0)
    print("Ready")

    while client.isConnected():

        curr_pos, curr_orien, _, curr_angles, _, _ = hand.get_state()


        socket_list = [sys.stdin, s]

        # Get the list sockets which are readable
        read_sockets, write_sockets, error_sockets = select.select(
            socket_list, [], [], 0)
        
        for sock in read_sockets:
            print("Incoming msg")
            #incoming message from remote server
            if sock is s:
                conn, add = s.accept()
                payload = conn.recv(4096)
                if not payload:
                    print('\nDisconnected from server')
                    break
                else:
                    payload = payload.decode("utf-8").rstrip("\x00")
                    payload = json.loads(payload)
                    print("received from client: {}".format(payload))

                    angles = np.array(payload["angles"])
                    position = np.array(payload["position"])
                    orientation = np.array(payload["orientation"])

                    # get collision data
                    data = np.zeros(21)
                    pts = client.getContactPoints(hand.body_id)
                    for pt in pts:
                        data[pt[3]] = 1
                    data = json.dumps(data.tolist(), ensure_ascii=False).encode('utf8')
                    conn.sendall(data)
        # # keep current position unless payload received
        # print(angles)            
        hand.set_target(position, orientation, angles)



if __name__ == "__main__":
    main()
