import numpy as np
import pybullet as pb
from pybullet_utils.bullet_client import BulletClient
import argparse
import socket


from mano_pybullet.hand_body import HandBody
from mano_pybullet.hand_model import HandModel20




# HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
# PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#     s.bind((HOST, PORT))
#     s.listen()
#     conn, addr = s.accept()
#     with conn:
#         print(f"Connected by {addr}")
#         while True:
#             data = conn.recv(1024)
#             if not data:
#                 break
#             conn.sendall(data)


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

    client.setRealTimeSimulation(True)

    # print(hand.joint_indices)
    # print(hand.joint_names)
    print(hand_model.link_names)
    while client.isConnected():

        position = np.zeros(3)
        # rotation = np.array([0,0,0,1])
        rotation = np.array([ 0.7071068, 0, 0, 0.7071068 ])
        angles = np.zeros(20)
        angles[0] = np.deg2rad(90)

        hand.set_target(position, rotation, angles)



if __name__ == "__main__":
    main()
