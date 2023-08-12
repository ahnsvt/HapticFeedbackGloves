# Libraries
from tkinter import * 
from tkinter import messagebox
import PIL
from PIL import Image,ImageTk
from numpy.lib.function_base import angle
import socket
import serial
import serial.tools.list_ports
from threading import Thread
from time import sleep, time
import numpy as np
import trace
import json
import sys
import os
import cv2
import math

np.set_printoptions(precision=2)

# creates a Tk() object
master = Tk()

#global variables
ready = False
arduino = ""
angles_raw = [0]*12
angles = [0]*16
mano_angles = [0]*16
unity = True
done = True
val = 0
sock = 0
s = 0 
button_web = 0

# Variables for calibration
angle_calibration_MCP = [0,45,80]
angle_calibration_PIP = [0,45,90]
x_MCP = [[0] * 3 for i1 in range(4)]
x_PIP = [[0] * 3 for i1 in range(4)]
z_MCP = [0]*4
coeff_MCP = [[0] * 3 for i1 in range(4)]
coeff_PIP = [[0] * 3 for i1 in range(4)]

#webcam variables
color_prox = (255, 255, 0) 
color_meta = (0, 255, 0)
color_hand = (0, 0, 255)
alpha = 0.4
thickness = 10
show_cam = False
cap = 0

#----------------------------function/thread for the interface--------------------------#
# 1) Main page: asking for the Port to which the Arduino is connected
def interface():
    global master
    master.title("H.E.D.A.S (Hand Exoskeleton Data Acquisition System)")
    master.state('normal')
    master.config(bg = 'medium aquamarine')

    ports = serial.tools.list_ports.comports(include_links=False) #looking for the acive ports

    if (len(ports) != 0): # we found a port
        label = Label(master, text ="Choose the port to which the device is connected:", font=("Abadi MT Condensed Extra Bold", 30), bg = 'medium aquamarine').pack(pady = 150)
        for port in ports :  # we display the ports' name on buttons
            Button(master, text = port.device, command = lambda : Calibration(port.device), font=("Abadi MT Condensed Extra Bold", 30), bg = 'snow').pack(pady = 20)
    else:  # else we need to connect the device and relaunch the program
        label = Label(master, text = "No port found. Connect a device and relaunch the program", font=("Abadi MT Condensed Extra Bold", 30), bg = 'medium aquamarine').pack(pady = 300)

# 2) Page where there is a choice between calibrating and using a previous calibration      
def Calibration(portDevice):
    global master,ready,arduino
    arduino = serial.Serial(port=portDevice, baudrate=115200, timeout=.1) # the arduino is connected to Python
    ready = True # the read_function can start reading the Arduino data
    eraseWidget()
    # checking if a file exists
    if os.path.isfile(os.path.join(path + os.sep, "sample.json")):
        Label(master, text ="Do you want to calibrate or use a previous calibration profile?", font=("Abadi MT Condensed Extra Bold", 30), bg = 'medium aquamarine').pack(pady = 150)
        Button(master, text = "New calibration", command = calibration, font=("Abadi MT Condensed Extra Bold", 30), bg = 'snow').pack(pady = 10)
        Button(master, text = "Use a user calibration profile", command = NoCal, font=("Abadi MT Condensed Extra Bold", 30), bg = 'snow').pack(pady = 50)
    else:
        Label(master, text ="You didn't calibrate yet... Create a new calibration", font=("Abadi MT Condensed Extra Bold", 30), bg = 'medium aquamarine').pack(pady = 150)
        Button(master, text = "New calibration", command = calibration, font=("Abadi MT Condensed Extra Bold", 30), bg = 'snow').pack(pady = 10)

# 3) Calibration process with 5 steps
def calibration():
    global master, val, show_cam,cap,thread1, button_web
    eraseWidget()
    #print(val)

    if val == 0:
        Label(master, text ="Put your hand flat with the thumb as far as possible from the fingers", font=("Abadi MT Condensed Extra Bold", 30), bg = 'medium aquamarine').pack()
        Label(master, text ="And your hand must be aligned with your arm", font=("Abadi MT Condensed Extra Bold", 30), bg = 'medium aquamarine').pack()
        image1 = Image.open(os.path.join(path + os.sep, "image","open_profile.png"))
        image2 = Image.open(os.path.join(path + os.sep, "image","open_face.png"))
        test1 = ImageTk.PhotoImage(image1)
        label1 = Label(image=test1, bg = 'medium aquamarine')
        label1.image = test1
        test2 = ImageTk.PhotoImage(image2)
        label2 = Label(image=test2, bg = 'medium aquamarine')
        label2.image = test2
        label1.place(x=225,y = master.winfo_height()/2,anchor = CENTER)
        label2.place(x=650,y = master.winfo_height()/2,anchor = CENTER)
        if not show_cam:
            Button(master, text = "Show the webcam", command = lambda : show(), font=("Abadi MT Condensed Extra Bold", 30), bg = 'snow').place(x=1200,y = master.winfo_height()/2,anchor = CENTER)
            Label(master, text ="This will return the video of your webcam (if you have a one).", font=("Abadi MT Condensed Extra Bold", 15), bg = 'medium aquamarine').place(x=1200,y = master.winfo_height()/2+55,anchor = CENTER)
            Label(master, text ="The same indicator as in the picture will show on the video", font=("Abadi MT Condensed Extra Bold", 15), bg = 'medium aquamarine').place(x=1200,y = master.winfo_height()/2+95,anchor = CENTER)
            Label(master, text ="to help you positioning your fingers.", font=("Abadi MT Condensed Extra Bold", 15), bg = 'medium aquamarine').place(x=1200+25,y = master.winfo_height()/2+135,anchor = CENTER)
            Label(master, text ="Opening the webcam can take a couple of seconds", font=("Abadi MT Condensed Extra Bold", 15), bg = 'medium aquamarine').place(x=1200+25,y = master.winfo_height()/2+175,anchor = CENTER)
   
    if val == 1:
        Label(master, text ="Fold your fingers in the same position as in the picture", font=("Abadi MT Condensed Extra Bold", 30), bg = 'medium aquamarine').pack()
        image1 = Image.open(os.path.join(path + os.sep,"image", "45_finger.png"))
        image2 = Image.open(os.path.join(path + os.sep,"image", "45_thumb.png"))
        test1 = ImageTk.PhotoImage(image1)
        label1 = Label(image=test1, bg = 'medium aquamarine')
        label1.image = test1
        test2 = ImageTk.PhotoImage(image2)
        label2 = Label(image=test2, bg = 'medium aquamarine')
        label2.image = test2
        label1.place(x=225,y = master.winfo_height()/2,anchor = CENTER)
        label2.place(x=650,y = master.winfo_height()/2,anchor = CENTER)
        if not show_cam:
            Button(master, text = "Show the webcam", command = lambda : show(), font=("Abadi MT Condensed Extra Bold", 30), bg = 'snow').place(x=1200,y = master.winfo_height()/2,anchor = CENTER)
            Label(master, text ="This will return the video of your webcam (if you have a one).", font=("Abadi MT Condensed Extra Bold", 15), bg = 'medium aquamarine').place(x=1200,y = master.winfo_height()/2+55,anchor = CENTER)
            Label(master, text ="The same indicator as in the picture will show on the video", font=("Abadi MT Condensed Extra Bold", 15), bg = 'medium aquamarine').place(x=1200,y = master.winfo_height()/2+95,anchor = CENTER)
            Label(master, text ="to help you positioning your fingers.", font=("Abadi MT Condensed Extra Bold", 15), bg = 'medium aquamarine').place(x=1200+25,y = master.winfo_height()/2+135,anchor = CENTER)
            Label(master, text ="Opening the webcam can take a couple of seconds", font=("Abadi MT Condensed Extra Bold", 15), bg = 'medium aquamarine').place(x=1200+25,y = master.winfo_height()/2+175,anchor = CENTER)
    
    if val == 2:
        Label(master, text = "Fold your fingers in the same position as in the picture", font=("Abadi MT Condensed Extra Bold", 30), bg = 'medium aquamarine').pack()
        image1 = Image.open(os.path.join(path + os.sep,"image", "90_finger.png"))
        image2 = Image.open(os.path.join(path + os.sep,"image", "90_thumb.png"))
        test1 = ImageTk.PhotoImage(image1)
        label1 = Label(image=test1, bg = 'medium aquamarine')
        label1.image = test1
        test2 = ImageTk.PhotoImage(image2)
        label2 = Label(image=test2, bg = 'medium aquamarine')
        label2.image = test2
        label1.place(x=225,y = master.winfo_height()/2,anchor = CENTER)
        label2.place(x=650,y = master.winfo_height()/2,anchor = CENTER)
        if not show_cam:
            Button(master, text = "Show the webcam", command = lambda : show(), font=("Abadi MT Condensed Extra Bold", 30), bg = 'snow').place(x=1200,y = master.winfo_height()/2,anchor = CENTER)
            Label(master, text ="This will return the video of your webcam (if you have a one).", font=("Abadi MT Condensed Extra Bold", 15), bg = 'medium aquamarine').place(x=1200,y = master.winfo_height()/2+55,anchor = CENTER)
            Label(master, text ="The same indicator as in the picture will show on the video", font=("Abadi MT Condensed Extra Bold", 15), bg = 'medium aquamarine').place(x=1200,y = master.winfo_height()/2+95,anchor = CENTER)
            Label(master, text ="to help you positioning your fingers.", font=("Abadi MT Condensed Extra Bold", 15), bg = 'medium aquamarine').place(x=1200+25,y = master.winfo_height()/2+135,anchor = CENTER)
            Label(master, text ="Opening the webcam can take a couple of seconds", font=("Abadi MT Condensed Extra Bold", 15), bg = 'medium aquamarine').place(x=1200+25,y = master.winfo_height()/2+175,anchor = CENTER)
   
    if val == 3:
        if show_cam:
            cap.release()
            thread1.kill()
        Label(master, text="Enter your calibration name", font=("Abadi MT Condensed Extra Bold", 40), bg = 'medium aquamarine').pack(pady = 300)
        entry1 = Entry(master, font=("Abadi MT Condensed Extra Bold", 30), bg = 'medium aquamarine')
        entry1.place(x= master.winfo_width()/2, y = master.winfo_height()/2,anchor = CENTER)
        Button(master, text = "Submit!", command = lambda : getName(entry1.get()), font=("Abadi MT Condensed Extra Bold", 30), bg = 'snow').place(x= master.winfo_width()/2, y = master.winfo_height()/2+100,anchor = CENTER)
        
    if val<3:
        Button(master, text = "Done!", command = lambda : get_calibration(), font=("Abadi MT Condensed Extra Bold", 30), bg = 'snow').place(x=725, y=700)

# 3.1) Function that collects the data necessary to estimate the angle from the potentiometer values
def get_calibration():
    global angles_raw, val,coeff_MCP,coeff_PIP
    eraseWidget()
    if val==0:
        for i in range(0,4):
            z_MCP[i] = angles_raw[i+8]
    if val < 3 :
        for j in range(0,4):
            x_PIP[j][val] = angles_raw[j]
            x_MCP[j][val] = angles_raw[j+4]

    val = val + 1

    if val == 3:
        # for k in range(0,4):
        #     print(x_PIP[k])

        # for l in range(0,4):
        #     print(x_MCP[k])    

        for t in range(0,4):
            coeff_MCP[t] = np.polyfit(x_MCP[t],angle_calibration_MCP,2).tolist()
            coeff_PIP[t] = np.polyfit(x_PIP[t],angle_calibration_PIP,2).tolist()
    calibration() # it always goes back to the calibration function

# shows the webcam if activated
def show():
    global show_cam,cap
    show_cam = True
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
    thread1.start()
    calibration()

#3.2) Once the calibration is finished, the program needs to save the data in a JSON file
def getName(name):
    global coeff_MCP, coeff_PIP, z_MCP, coeff_wrist
    if os.path.isfile(os.path.join(path + os.sep, "sample.json")): # it verifies if a file already exists 
        with open(os.path.join(path + os.sep, "sample.json")) as json_open: # if there exists one, it opens it
            data = json.load(json_open)
            
        data['Usernames'].append({'name': name,'PIP': coeff_PIP,'MPCx': coeff_MCP,'MPCz': z_MCP})

        with open(os.path.join(path + os.sep, "sample.json"),"w") as json_write:
            json.dump(data,json_write)

    else: # otherwise we create a file in the directory of the program
        data = {}
        data['Usernames'] = []
        data['Usernames'].append({'name': name,'PIP': coeff_PIP,'MPCx': coeff_MCP,'MPCz': z_MCP})

        with open(os.path.join(path + os.sep, "sample.json"), "w") as json_create:
            json.dump(data, json_create)

    final_page() # Finally, the code arrives to the final page

# 4) It is called if the user choose to use a previous calibration
def NoCal():
    eraseWidget()
    Label(master, text ="Choose your calibration profile", font=("Abadi MT Condensed Extra Bold", 30), bg = 'medium aquamarine').pack(pady = 100)
    with open(os.path.join(path + os.sep, "sample.json")) as json_open:
        data = json.load(json_open)
    for user in data['Usernames'] : 
        Button(master, text = user['name'], command = lambda username = user: get_coeff(username['PIP'],username['MPCx'],username['MPCz']), font=("Abadi MT Condensed Extra Bold", 30), bg = 'snow').pack(pady = 10)

# Function that replace the coefficient from the JSON

def get_coeff(PIP, MPCx, MPCz):
    global coeff_MCP, coeff_PIP, z_MCP
    coeff_PIP = PIP
    coeff_MCP = MPCx
    z_MCP = MPCz
    final_page()

# 5) The final page just display a message (and decides if a connection to Unity occurs)
# A function can be added here with a thread to be used for anything
def final_page():
    #Unity() #comment this line if you don't want a connection to Unity
    eraseWidget()
    Label(master, text ="You are ready to use the glove", font=("Abadi MT Condensed Extra Bold", 30), bg = 'medium aquamarine').pack(pady = 150)
    Label(master, text ="The angles are avaibles in the array 'angles'", font=("Abadi MT Condensed Extra Bold", 30), bg = 'medium aquamarine').pack(pady = 10)
    Label(master, text ="Or try in UNITY", font=("Abadi MT Condensed Extra Bold", 30), bg = 'medium aquamarine').pack()

#----A class that allows a function/thread to be closed if it's an infinity loop---#
class thread_with_trace(Thread):
  def __init__(self, *args, **keywords):
    Thread.__init__(self, *args, **keywords)
    self.killed = False
  
  def start(self):
    self.__run_backup = self.run
    self.run = self.__run      
    Thread.start(self)
  
  def __run(self):
    sys.settrace(self.globaltrace)
    self.__run_backup()
    self.run = self.__run_backup
  
  def globaltrace(self, frame, event, arg):
    if event == 'call':
      return self.localtrace
    else:
      return None
  
  def localtrace(self, frame, event, arg):
    if self.killed:
      if event == 'line':
        raise SystemExit()
    return self.localtrace
  
  def kill(self):
    self.killed = True

#---------------function that treats, arranges and send the data----------------#
def read_function():
    global ready,angles_raw,angles,unity,sock,s
    while True:
        if ready == True:
            data = arduino.readline()[:-2]

            if data and len(data) == 83:
                angles_raw = [float(x) for x in data.split()]
                print("----")
                print("Angles Raw: {}".format(angles_raw))
                for i in range (0,4):
                    # Index to Pinkie PIPs
                    angles[i] = poly_reg(coeff_PIP[i], angles_raw[i])
                    # MCPx
                    angles[i+4]= poly_reg(coeff_MCP[i], angles_raw[i+4])

                
                for i in range (0,4):
                    # MCPz
                    angles[i+8] = potToAngle(z_MCP[i]) - potToAngle(angles_raw[i+8])

                for i in range (0,4): 
                    # DIP
                    angles[i+12] = angles[i]*0.66 #Might need to change this eqn based on which DIP-PIP relationship I go with
                mano_angles = np.zeros(16)
                for i in range (0,4):
                    for j in range (0,4):
                        if i < 2:
                            if j == 0:
                                mano_angles[4*i + j + 2] = angles[4*j + i] 
                            elif j == 2:
                                mano_angles[4*i + j - 2] = angles[4*j + i]
                            else:
                                mano_angles[4*i + j] = angles[4*j + i]

                        elif i == 2:
                            if j == 0:
                                mano_angles[4*i + j + 2] = angles[4*j + i + 1]
                            elif j == 2:
                                mano_angles[4*i + j - 2] = angles[4*j + i + 1]
                            else:
                                mano_angles[4*i + j] = angles[4*j + i + 1]

                        elif i == 3:
                            if j == 0:
                                mano_angles[4*i + j + 2] = angles[4*j + i - 1]
                            elif j == 2:
                                mano_angles[4*i + j - 2] = angles[4*j + i - 1]
                            else:
                                mano_angles[4*i + j] = angles[4*j + i - 1]

                # for i in range (0,16):
                mano_angles = np.array(mano_angles)
                mano_angles_sock = np.zeros(20)
                mano_angles_sock[:16] = np.deg2rad(mano_angles)
                print("Angles: {}".format(mano_angles_sock))

                if unity:
                    HOST = "127.0.0.1"  # The server's hostname or IP address
                    PORT = 65430  # The port used by the server
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                        s.connect((HOST, PORT))
                        position = np.array([0,0,0.35])
                        orientation = np.array([ 0.7071068, 0, 0, 0.7071068 ])
                        data = dict()
                        data["angles"] = mano_angles_sock.tolist()
                        data["position"] = position.tolist()
                        data["orientation"] = orientation.tolist()
                        data = json.dumps(data, ensure_ascii=False).encode('utf8')
                        #s.sendall(str(angles[7]).encode('utf-8'))
                        # for i in range (0,23):
                        #     angles[i] = int(angles[i]*1000)
                        s.sendall(data)
                        payload = s.recv(4096)
                        contact_info = json.loads(payload.decode("utf-8").rstrip("\x00"))

                        # Pybullet will return an array of 21 bits indicating collision (1) or no collision (0) of each link. 
                        # The indexing of the array and the corresponding links are as follows, where 1 = Proximal, 2 = Intermediate, 3 = Distal phalanges:

                        # 0 = palm
                        # 1 = index1 y
                        # 2 = index1 x
                        # 3 = index2
                        # 4 = index3
                        # 5 = middle1 y
                        # 6 = middle1 x
                        # 7 = middle2
                        # 8 = middle3
                        # 9 = pinky1 y
                        # 10 = pinky1 x
                        # 11 = pinky2
                        # 12 = pinky3
                        # 13 = ring1 y
                        # 14 = ring1 x
                        # 15 = ring2
                        # 16 = ring3
                        # 17 = thumb1 y
                        # 18 = thumb1 z
                        # 19 = thumb2
                        # 20 = thumb3

                        # unwanted = [0,1,4,5,8,9,12,13,16,17,18,19,20] # Unwanted contact list indices
                        wanted = [2,3,6,7,10,11,14,15]
                        contact_info_truncated = [int(contact_info[idx]) for idx in wanted]
                        print("Contact info: {}".format(contact_info_truncated))
                        arduino.write(contact_info_truncated)
                
def show_image():
    global show_cam,val,cap
    while True:
        if show_cam:
            _, frame = cap.read()
            frame = cv2.flip(frame, 1)
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
            overlay = cv2image.copy()
            if val == 0:
                cv2.line(overlay, (400,450), (400,300), color_hand, 30)
                cv2.line(overlay, (400,300), (400,200), color_meta, 30)
            if val == 1:
                angles1 = math.pi/4
                angles2 = math.pi/4
                cv2.line(overlay, (400,400), (400,300), color_hand, thickness)
                cv2.line(overlay, (400,300), (int(400+50*math.cos(angles1)),int(300-50*math.sin(angles1))), color_meta, thickness)
                cv2.line(overlay, (int(400+50*math.cos(angles1)),int(300-50*math.sin(angles1))), (int(400+50*math.cos(angles1)+50*math.sin(angles2+angles1)),int(300-50*math.sin(angles1)-50*math.cos(angles2+angles1))), color_prox, thickness)
                angles1 = math.pi/4*3
                angles2 = math.pi/4*3
                cv2.line(overlay, (400,300), (int(400+50*math.cos(angles1)),int(300-50*math.sin(angles1))), color_meta, thickness)
                cv2.line(overlay, (int(400+50*math.cos(angles1)),int(300-50*math.sin(angles1))), (int(400+50*math.cos(angles1)+50*math.sin(angles2+angles1)),int(300-50*math.sin(angles1)-50*math.cos(angles2+angles1))), color_prox, thickness)
            if val == 2:
                angles1 = math.radians(0)
                angles2 = math.radians(-90)
                cv2.line(overlay, (400,400), (400,300), color_hand, thickness)
                cv2.line(overlay, (400,300), (int(400+50*math.cos(angles1)),int(300-50*math.sin(angles1))), color_meta, thickness)
                cv2.line(overlay, (int(400+50*math.cos(angles1)),int(300-50*math.sin(angles1))), (int(400+50*math.cos(angles1)+50*math.cos(angles2+angles1)),int(300-50*math.sin(angles1)-50*math.sin(angles2+angles1))), color_prox, thickness)
                angles1 = math.radians(180)
                angles2 = math.radians(90)
                cv2.line(overlay, (400,400), (400,300), color_hand, thickness)
                cv2.line(overlay, (400,300), (int(400+50*math.cos(angles1)),int(300-50*math.sin(angles1))), color_meta, thickness)
                cv2.line(overlay, (int(400+50*math.cos(angles1)),int(300-50*math.sin(angles1))), (int(400+50*math.cos(angles1)+50*math.cos(angles2+angles1)),int(300-50*math.sin(angles1)-50*math.sin(angles2+angles1))), color_prox, thickness)

            img = PIL.Image.fromarray(cv2image)
            imgtk = ImageTk.PhotoImage(image=img)
            label2 = Label(image=imgtk, bg = 'medium aquamarine')
            label2.image = imgtk
            label2.place(x=1200,y = master.winfo_height()/2,anchor = CENTER)

#----------general functions--------# 
# when the close button is pressed on the window
def on_closing():
    global cap,show_cam
    if messagebox.askokcancel("Quit", "Do you want to quit?"):
        if show_cam:
            cv2.destroyAllWindows()
            thread1.kill()
        thread.kill()
        master.quit()

# function that erase every widget on the screen
def eraseWidget():
    for widgets in master.winfo_children():
        widgets.destroy()

# takes the coeff for argument, return the values to a second degree function
def poly_reg(coeff,val):
    return coeff[0] * pow(val,2) + coeff[1]*val + coeff[2]

# function that maps the potentiometer value to an angle (the pot goes from 15° to 345°, ADC val = 0  to 1023)
def potToAngle(val):
    return val * 330 / 1023 + 15

# Main where the Threads are started
if __name__ == "__main__":
    thread = thread_with_trace(target = read_function) # "       " read the values, process it and send it with Socket
    thread1 = thread_with_trace(target = show_image) # define the thread/function that shows the webcam
    thread2 = Thread(target = interface) # "       " shows the interface with Tkinter

    #starts the threads 0 and 1
    thread2.start()
    thread.start()
    path = os.path.dirname(os.path.abspath(__file__))
    master.protocol("WM_DELETE_WINDOW", on_closing)
    master.mainloop()