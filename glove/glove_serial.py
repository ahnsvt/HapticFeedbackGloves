""" 
Function
- calibrate glove using existing work
- connect with mano_pybullet env
loop
- extract angles
- set angles in env
- get collision info from env
- transfer back to arduino
end loop
"""


"""
TODO
- Port calibration seq
- Port angle extraction seq
- Build pybullet env
    - needs the MANO hand with 20DOF
    - limit the thumb info (we only have 18 DOF)
    - Add an object to interact with
    - Add collision detection routine
"""