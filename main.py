# ME 462 Term Project Graphical User Interface
# Prepared by Ermek Belekov and Emre Gonen 
# Mentor AHMET BUGRA KOKU 


# Do imports
from tkinter import *
import tkinter.messagebox
from numpy import *
import warnings
import serial
import time
import struct

# Some Initiations

# Initiate serial protocol
arduino = serial.Serial('/dev/ttyUSB1', 9600)

# To raise RunTimeWarning, implement it as error
warnings.simplefilter('error', 'RunTimeWarning')

# Some Constants
centerServo = (55, 55)
radiusServo = 45

'''
 ServoAngles function which takes desired x, y, z, pitch, roll and yaw values in meters and radians, 
 and convert them into angle of servo motors in radians by using Inverse Kinematics. The equations and implementations
 are used as presented in http://www.instructables.com/id/Stewart-Platform/ . See the link for more information.
'''

def ServoAngles(x, y, z, pitch, roll, yaw):
    # Angular Coordinates of Servo Motors on Base [deg]
    beta_b = array([-30, -90, -150, 150, 90, 30])
    beta_b = beta_b * pi / 180  # Converting into radians

    # Angular Coordinates of Joints on Upper Plate [deg]
    beta_p = array([-15, -105, -135, 135, 105, 15])
    beta_p = beta_p * pi / 180  # Converting into radians

    # Initial Position of Servo Arms
    beta = array([-150, 30, 90, -90, -30, 150])
    beta = beta * pi / 180  # Converting into radians

    # Outer Tangent Circle Radius of Base [m]
    Rb = 0.238

    # Outer Tangent Circle Radius of Upper Plate [m]
    Rp = 0.19847 / 2

    # Servo Operating Arm [m]
    a = 0.05

    # Length of Rods [m]
    s = 0.3

    # Servo Arm Position Matrix at Initial
    b = array([[Rb * cos(beta_b[0]), Rb * sin(beta_b[0]), 0],
               [Rb * cos(beta_b[1]), Rb * sin(beta_b[1]), 0],
               [Rb * cos(beta_b[2]), Rb * sin(beta_b[2]), 0],
               [Rb * cos(beta_b[3]), Rb * sin(beta_b[3]), 0],
               [Rb * cos(beta_b[4]), Rb * sin(beta_b[4]), 0],
               [Rb * cos(beta_b[5]), Rb * sin(beta_b[5]), 0]])

    b = b.T  # Taking Transpose of it

    # Extracting Positions in x,y,z coordinates
    xb = b[0]
    yb = b[1]
    zb = b[2]

    # Edge Points of Upper Plate Position Matrix at Initial
    p = array([[Rp * cos(beta_p[0]), Rp * sin(beta_p[0]), 0],
               [Rp * cos(beta_p[1]), Rp * sin(beta_p[1]), 0],
               [Rp * cos(beta_p[2]), Rp * sin(beta_p[2]), 0],
               [Rp * cos(beta_p[3]), Rp * sin(beta_p[3]), 0],
               [Rp * cos(beta_p[4]), Rp * sin(beta_p[4]), 0],
               [Rp * cos(beta_p[5]), Rp * sin(beta_p[5]), 0]])

    p = p.T  # Taking Transpose of it

    # Extracting Positions in x,y,z coordinates
    xp = p[0]
    yp = p[1]
    zp = p[2]

    # Setting Home Position where servo arms are at horizontal position and z-axis value is zero
    h0 = sqrt(a ** 2 + s ** 2 - (xp - xb) ** 2 - (yp - yb) ** 2) - zp
    h0 = h0[0]

    theta = pitch
    phi = roll
    psi = yaw

    # Base to Upper Plate Rotation Matrix
    Rpb = array([[cos(psi) * cos(theta), cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi),
                  sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)],
                 [cos(theta) * sin(psi), cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta),
                  cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi)],
                 [-sin(theta), cos(theta) * sin(phi), cos(phi) * cos(theta)]])

    # Effective Leg Lengths
    tt = array([[x], [y], [h0 + z]])

    # Platform Coordinates in Base Framework
    q = tile(tt, 6) + dot(Rpb, p)

    # Extracting Positions in x,y,z coordinates
    xq = q[0]
    yq = q[1]
    zq = q[2]

    # Leg Lengths
    l = q - b

    # Calculating RC Servo Motors' angles. If there is an error, it returns a list contains of all 8s,
    # which is chosen arbitrarily

    try:
        # Servo Angles
        L = sum(multiply(l, l), axis=0) - (s ** 2 - a ** 2)
        M = 2 * a * (zq - zb)
        N = 2 * a * (multiply(cos(beta), (xq - xb)) + multiply(sin(beta), (yq - yb)))
        alpha = arcsin(divide(L, sqrt(M ** 2 + N ** 2))) - arctan(divide(N, M))
    except:
        return [8, 8, 8, 8, 8, 8]

    return alpha

'''
 MaxRange Function is used to calculate the maximum upper value of an input. In other words, if one wants to find
 the upper limit of pitch value with respect to other values such as x=20mm, y=36mm, z=-5mm, roll=1deg, yaw=-25deg,
 it is found as 33.63 degree. Please note that, when other values changes upper limit is changes.
 
 To use the function variable_index must be an integer between 0 and 5. These number is used to find the upper limit of
 
        0: x
        1: y
        2: z
        3: pitch
        4: roll
        5: yaw
        
 The other inputs of the function which are named as d1, d2, d3, d4 and d5 are ordered respectively. For example,
 
        MaxRange(4, 0.002, -0.010, -0.002, 0.170, -0.130) returns
        
        maximum upper limit of Roll value at x=0.002m, y=-0.010m, z=-0.002m, pitch=0.170rad and yaw=-0.130rad
        which is 24.75 deg
 '''

def MaxRange(variable_index, d1, d2, d3, d4, d5):

    if variable_index == 0:
        for i in arange(0, 0.1, 0.001):
            if isin(8, ServoAngles(i, d1, d2, d3, d4, d5)).any():
                return (i-0.01)*1000

    elif variable_index == 1:
        for i in arange(0, 0.1, 0.001):
            if isin(8, ServoAngles(d1, i, d2, d3, d4, d5)).any():
                return (i-0.01)*1000

    elif variable_index == 2:
        for i in arange(0, 0.1, 0.001):
            if isin(8, ServoAngles(d1, d2, i, d3, d4, d5)).any():
                return (i-0.01)*1000

    elif variable_index == 3:
        for i in arange(0, 1, 0.001):
            if isin(8, ServoAngles(d1, d2, d3, i, d4, d5)).any():
                return (i-0.01)*180/pi

    elif variable_index == 4:
        for i in arange(0, 1, 0.001):
            if isin(8, ServoAngles(d1, d2, d3, d4, i, d5)).any():
                return (i-0.01)*180/pi

    elif variable_index == 5:
        for i in arange(0, 1, 0.001):
            if isin(8, ServoAngles(d1, d2, d3, d4, d5, i)).any():
                return (i-0.01)*180/pi


'''
 MinRange Function, similar as MaxRange, is used to calculate the minimum lower value of an input. For example,

        MaxRange(4, 0.002, -0.010, -0.002, 0.170, -0.130) returns

        maximum upper limit of Roll value at x=0.002m, y=-0.010m, z=-0.002m, pitch=0.170rad and yaw=-0.130rad
        which is -24.75 deg
 '''

def MinRange(variable_index, d1, d2, d3, d4, d5):

    if variable_index == 0:
        for i in arange(0, -0.1, -0.001):
            if isin(8, ServoAngles(i, d1, d2, d3, d4, d5)).any():
                return (i + 0.01) * 1000

    elif variable_index == 1:
        for i in arange(0, -0.1, -0.001):
            if isin(8, ServoAngles(d1, i, d2, d3, d4, d5)).any():
                return (i + 0.01) * 1000

    elif variable_index == 2:
        for i in arange(0, -0.1, -0.001):
            if isin(8, ServoAngles(d1, d2, i, d3, d4, d5)).any():
                return (i + 0.01) * 1000

    elif variable_index == 3:
        for i in arange(0, -1, -0.001):
            if isin(8, ServoAngles(d1, d2, d3, i, d4, d5)).any():
                return (i + 0.01) * 180 / pi

    elif variable_index == 4:
        for i in arange(0, -1, -0.001):
            if isin(8, ServoAngles(d1, d2, d3, d4, i, d5)).any():
                return (i + 0.01) * 180 / pi

    elif variable_index == 5:
        for i in arange(0, -1, -0.001):
            if isin(8, ServoAngles(d1, d2, d3, d4, d5, i)).any():
                return (i + 0.01) * 180 / pi


'''
 Main duty of getThrottle function is to handle with widgets of the gui. If one uses buttons, this triggers this
 function to calculate required motor angles, send them into serial port and update the gui.
'''
def getThrottle(angles):
    angles_inDeg = angles * 180 /pi

    # Then update the gui.

    # Update the current position
    LabelX.configure(text=("X: " + str(var_x.get()) + "mm"))
    LabelY.configure(text=("Y: " + str(var_y.get()) + "mm"))
    LabelZ.configure(text=("Z: " + str(var_z.get()) + "mm"))
    LabelP.configure(text=("Pitch: " + str(var_pitch.get()) + "deg"))
    LabelR.configure(text=(" Roll: " + str(var_roll.get()) + "deg"))
    LabelYa.configure(text=("  Yaw: " + str(var_yaw.get()) + "deg"))

    # Update the arrow angles
    servo1.coords(line1, centerServo[0], centerServo[1], centerServo[0]-radiusServo*cos(angles[0]), centerServo[1]-radiusServo*sin(angles[0]))
    servo2.coords(line2, centerServo[0], centerServo[1], centerServo[0]+radiusServo*cos(angles[1]), centerServo[1]+radiusServo*sin(angles[1]))
    servo3.coords(line3, centerServo[0], centerServo[1], centerServo[0]-radiusServo*cos(angles[2]), centerServo[1]-radiusServo*sin(angles[2]))
    servo4.coords(line4, centerServo[0], centerServo[1], centerServo[0]+radiusServo*cos(angles[3]), centerServo[1]+radiusServo*sin(angles[3]))
    servo5.coords(line5, centerServo[0], centerServo[1], centerServo[0]-radiusServo*cos(angles[4]), centerServo[1]-radiusServo*sin(angles[4]))
    servo6.coords(line6, centerServo[0], centerServo[1], centerServo[0]+radiusServo*cos(angles[5]), centerServo[1]+radiusServo*sin(angles[5]))

    # Update the angles display
    servo1.itemconfig(text1, text='{0:.1f}'.format(180 - angles_inDeg[0]))
    servo2.itemconfig(text2, text='{0:.1f}'.format(-angles_inDeg[1]))
    servo3.itemconfig(text3, text='{0:.1f}'.format(180 - angles_inDeg[2]))
    servo4.itemconfig(text4, text='{0:.1f}'.format(-angles_inDeg[3]))
    servo5.itemconfig(text5, text='{0:.1f}'.format(180 - angles_inDeg[4]))
    servo6.itemconfig(text6, text='{0:.1f}'.format(-angles_inDeg[5]))

'''
 SerialWriter function is used to send the motor positions to arduino in proper way. It takes angles in deg, arranges
 them into proper angles because position of servo arms different. Note that it sends 5 significant figure angle value
 to arduino such as 107.11 to first motor.
'''
def SerialWriter(angles):

    # print("ANG", angles)

    serialAngles = [90 - angles[0], 90 + angles[1], 90 - angles[2], 90 + angles[3], 90 - angles[4], 90 + angles[5]]

    print("SER", serialAngles)

    arduino.write(struct.pack('>12B',
                              int(serialAngles[0]), int(serialAngles[0]%1*10),
                              int(serialAngles[1]), int(serialAngles[1]%1*10),
                              int(serialAngles[2]), int(serialAngles[2]%1*10),
                              int(serialAngles[3]), int(serialAngles[3]%1*10),
                              int(serialAngles[4]), int(serialAngles[4]%1*10),
                              int(serialAngles[5]), int(serialAngles[5]%1*10)))

'''
 setZero function is a simple function. It sends all motors to their initial positions
'''
def setZero():
    var_x.set(0)
    var_y.set(0)
    var_z.set(0)
    var_pitch.set(0)
    var_roll.set(0)
    var_yaw.set(0)

def sinewave(i, stepSize, reqTime):

        # Transferring values from mm to m and deg to rad
        x_m = var_x.get()/1000*sin(i)
        y_m = var_y.get()/1000*sin(i)
        z_m = var_z.get()/1000*sin(i)
        pitch_m = var_pitch.get()*pi/180*sin(i)
        roll_m = var_roll.get()*pi/180*sin(i)
        yaw_m = var_yaw.get()*pi/180*sin(i)

        # print(x_m, y_m, z_m, pitch_m, roll_m, yaw_m)

        # Calculating the motor angles
        angles = ServoAngles(x_m, y_m, z_m, pitch_m, roll_m, yaw_m)

        if 8 not in angles:
            angles_inDeg = angles*180/pi        # If not, send them into SerialWriter function in angles
            SerialWriter(angles_inDeg)
            print(angles)
            getThrottle(angles)

            return False

        # If there is NAN term, send the user an
        # error message to say that change inputs
        else:
            tkinter.messagebox.showwarning("Servo Angles Exceed!", "You are out of working space!\n"
                                                               "Please change your inputs!")
            return True

def sendRaw():
    # Transferring values from mm to m and deg to rad
    x_m = var_x.get() / 1000
    y_m = var_y.get() / 1000
    z_m = var_z.get() / 1000
    pitch_m = var_pitch.get() * pi / 180
    roll_m = var_roll.get() * pi / 180
    yaw_m = var_yaw.get() * pi / 180

    # print(x_m, y_m, z_m, pitch_m, roll_m, yaw_m)

    # Calculating the motor angles
    angles = ServoAngles(x_m, y_m, z_m, pitch_m, roll_m, yaw_m)

    if 8 not in angles:
        angles_inDeg = angles * 180 / pi  # If not, send them into SerialWriter function in angles
        SerialWriter(angles_inDeg)
        getThrottle(angles)

    # If there is NAN term, send the user an error message to say that change inputs
    else:
        tkinter.messagebox.showwarning("Servo Angles Exceed!", "You are out of working space!\n"
                                                               "Please change your inputs!")

def SetStateOn(force = True):
    global running
    if force:
        running = True
    if running:
        t = linspace(0, var_Time.get(), var_StepSize.get())
        for i in t:
            cond = sinewave(i, var_StepSize.get(), var_Time.get())
            root.after(int(var_Time.get() / var_StepSize.get() * 1000))
            if cond:
                break

def IMUCalibrationStart():
    arduino.write('c'.encode())
    IMUCalibration()

def IMUCalibration():
    print(arduino.readline().decode())
    print(type(arduino.readline().decode()))
    if arduino.readline().decode() != "b":
        root.after(10, IMUCalibration())
    else:
        pass

def IMUSetZero():
    var_x.set(0)
    var_y.set(0)
    var_z.set(0)
    var_pitch.set(0)
    var_roll.set(0)
    var_yaw.set(0)
    sendRaw()

    time.sleep(0.1)
    packet = arduino.readline()
    packet.split(",")
    ax = packet[0]
    ay = packet[1]
    az = packet[2]
    gx = packet[3]
    gy = packet[4]
    gz = packet[5]

    var_IMU_x_acc.set(ax)
    var_IMU_y_acc.set(ay)
    var_IMU_z_acc.set(az)
    var_IMU_pitch.set(gy)
    var_IMU_roll.set(gz)
    var_IMU_yaw.set(gx)

    var_pitch.set(int(var_IMU_pitch)*-1)
    var_roll.set(int(var_IMU_roll) * -1)

    sendRaw()

def HELP():
    tkinter.messagebox.showinfo("How to use it?", "1. Zero All Button is to zero the upper plate to itself.\n"
                                                  "2. Calibrate IMU Button is to calibrate MPU 6050 device.\n"
                                                  "3. Zero to IMU Button is  zero to upper plate to absolute gravity.\n"
                                                  "4. Cartesian Jogging Menu is to increase or to decrease the platform in desired orientations.\n"
                                                  "5. You can change the increment value up to 10 mm using spinboxes.\n"
                                                  "6. Cartesian Trajectory Menu is to give a sinusoidal wave to at a desired time and step.\n"
                                                  "7. You can read acceleration values in x, y and z axis as well as pitch, roll and yaw angles from IMU.\n"
                                                  "8. You can read six input values from End Effector.\n"
                                                  "9. You can monitor current servo angles from RC Servo Motor Angles.\n"
                                                  "\n"
                                                  "\n"
                                               

#####################################    THIS PART IS FOR GUI    ###################################################

# First Initialize it
root = Tk()

# Define Variables that are used in it
var_x = DoubleVar(root)
var_y = DoubleVar(root)
var_z = DoubleVar(root)
var_pitch = DoubleVar(root)
var_roll = DoubleVar(root)
var_yaw = DoubleVar(root)

var_PosIncrement = StringVar(root)
var_OrIncrement = StringVar(root)

var_StepSize = IntVar(root)
var_Time = DoubleVar(root)

var_usbPort = StringVar(root)

var_IMU_x_acc = StringVar(root)
var_IMU_y_acc = StringVar(root)
var_IMU_z_acc = StringVar(root)
var_IMU_pitch = StringVar(root)
var_IMU_roll = StringVar(root)
var_IMU_yaw = StringVar(root)

'''
 The structure of the gui is in the following way:
    
    1. There are two divisions:             Left Frame and Right Frame
    2. Left Frame has two regions:          Left Upper Part (Home Button) and Left Lower Part (Cartesian Jogging)
    3. Left Upper Part has only a button to reset the motors
    4. Left Lower Part has two regions:     Translation Jogging and Rotation Jogging
    5. Each Jogging has four sub-frames containing a label, two increment buttons and an increment value
    6. Right Frame has two regions:         Right Upper Part (End Effector) and Right Lower Part (RC Motor Angles)
    7. Right Upper Part contains all labels to monitor current situation of the platform
    8. Right Lower Part contains six servo motors indicating their current positive angle values
'''

# Create Left Frame
frameLEFT = Frame(root)
frameLEFT.pack(side=LEFT)

# Create Left Upper Frame
frameLEFTUPPER = Frame(frameLEFT)
frameLEFTUPPER.pack(side=TOP)

# Create 1st Part of Left Upper Frame
frameLEFTUPPER_1 = Frame(frameLEFTUPPER)
frameLEFTUPPER_1.pack(side=LEFT)

# Create 2nd Part of Left Upper Frame
frameLEFTUPPER_2 = Frame(frameLEFTUPPER)
frameLEFTUPPER_2.pack(side=LEFT)

# Create Home Button
HomeButton = Button(frameLEFTUPPER_1, text="Zero All", command=lambda: [setZero(), sendRaw()], height=9, width=10)
HomeButton.pack(side=TOP, padx= 10, pady=10)

IMUCalibrateButton = Button(frameLEFTUPPER_2, text='Calibrate IMU', command=lambda: [IMUCalibrationStart()], height=3, width=10)
IMUCalibrateButton.pack(side=TOP, padx=10, pady=10)

IMUZeroButton = Button(frameLEFTUPPER_2, text='Zero to IMU', command=lambda: [IMUSetZero()], height=3, width=10)
IMUZeroButton.pack(side=TOP, padx=10, pady=10)

HelpButton = Button(frameLEFTUPPER, text='?', command= lambda: HELP(), height=9, width=10)
HelpButton.pack(side=LEFT, pady=10)

# Create Left Middle Frame
frameLEFTMIDDLE = Frame(frameLEFT)
frameLEFTMIDDLE.pack(side=TOP)

# Add Heading to Left Middle Frame
HeadingLEFTMIDDLE = Label(frameLEFTMIDDLE, text="Cartesian Jogging", font='Helvetica 18 bold').pack(side=TOP, fill=X)

# Create Left Lower Frame
frameLEFTLOWER = Frame(frameLEFT)
frameLEFTLOWER.pack(side=TOP)

# Add Heading to Left Lower Frame
HeadingLEFTLOWER = Label(frameLEFTLOWER, text="Cartesian Trajectory", font='Helvetica 18 bold').pack(side=TOP, fill=X, padx=10, pady=10)

# Create Translational Jogging Frame
frameTJ = Frame(frameLEFTMIDDLE)
frameTJ.pack(side=LEFT)

# Create Rotational Jogging Frame
frameRJ = Frame(frameLEFTMIDDLE)
frameRJ.pack(side=LEFT)

# Create X-Jogging
frameTJx = Frame(frameTJ)
frameTJx.pack(side=TOP)
# Initially set it as 0
var_x.set(0)
# Type its label
Lx = Label(frameTJx, text="X:").pack(side=LEFT, fill=X, padx=10, pady=10)
# Create Increment and Decrement buttons, and triggered these buttons to these functions
But1p = Button(frameTJx, text="+", command=lambda: [var_x.set(var_x.get() + float(var_PosIncrement.get())), sendRaw()]).pack(side=LEFT)
But1n = Button(frameTJx, text="-", command=lambda: [var_x.set(var_x.get() - float(var_PosIncrement.get())), sendRaw()]).pack(side=LEFT)

# Create Y-Jogging (Operations are same as X-Jogging)
frameTJy = Frame(frameTJ)
frameTJy.pack(side=TOP)
var_y.set(0)
Ly = Label(frameTJy, text="Y:").pack(side=LEFT, fill=X, padx=10, pady=10)
But2p = Button(frameTJy, text="+", command=lambda: [var_y.set(var_y.get() + float(var_PosIncrement.get())), sendRaw()]).pack(side=LEFT)
But2n = Button(frameTJy, text="-", command=lambda: [var_y.set(var_y.get() - float(var_PosIncrement.get())), sendRaw()]).pack(side=LEFT)

# Create Z-Jogging (Operations are same as X-Jogging)
frameTJz = Frame(frameTJ)
frameTJz.pack(side=TOP)
var_z.set(0)
Lz = Label(frameTJz, text="Z:").pack(side=LEFT, fill=X, padx=10, pady=10)
But3p = Button(frameTJz, text="+", command=lambda: [var_z.set(var_z.get() + float(var_PosIncrement.get())), sendRaw()]).pack(side=LEFT)
But3n = Button(frameTJz, text="-", command=lambda: [var_z.set(var_z.get() - float(var_PosIncrement.get())), sendRaw()]).pack(side=LEFT)

# Create Interval of Change Spinbox for Translational Jogging
frameTJsb = Frame(frameTJ)
frameTJsb.pack(side=TOP)
# Set it as 2.00 as default
var_PosIncrement.set("2.00")
# Create the spinbox for range between 0 and 10
PosIncrement = Spinbox(frameTJsb, textvariable=var_PosIncrement, from_=0, to=10, justify=RIGHT, width=5, format='%.2f')
PosIncrement.pack(side=TOP, padx=10, pady=5)
# Type the Label of it
LPosInc = Label(frameTJsb, text="Position Increment\n[mm]").pack(side=TOP, padx=10)

# Create Pitch-Jogging (Operations are same as X-Jogging)
frameRJp = Frame(frameRJ)
frameRJp.pack(side=TOP)
var_pitch.set(0)
Lp = Label(frameRJp, text="Pitch:").pack(side=LEFT, fill=X, padx=10, pady=10)
But4p = Button(frameRJp, text="+", command=lambda: [var_pitch.set(var_pitch.get() + float(var_OrIncrement.get())), sendRaw()]).pack(side=LEFT)
But4n = Button(frameRJp, text="-", command=lambda: [var_pitch.set(var_pitch.get() - float(var_OrIncrement.get())), sendRaw()]).pack(side=LEFT)

# Create Roll-Jogging (Operations are same as X-Jogging)
frameRJr = Frame(frameRJ)
frameRJr.pack(side=TOP)
var_roll.set(0)
Lr = Label(frameRJr, text=" Roll:").pack(side=LEFT, fill=X, padx=10, pady=10)
But5p = Button(frameRJr, text="+", command=lambda: [var_roll.set(var_roll.get() + float(var_OrIncrement.get())), sendRaw()]).pack(side=LEFT)
But5n = Button(frameRJr, text="-", command=lambda: [var_roll.set(var_roll.get() - float(var_OrIncrement.get())), sendRaw()]).pack(side=LEFT)

# Create Yaw-Jogging (Operations are same as X-Jogging)
frameRJy = Frame(frameRJ)
frameRJy.pack(side=TOP)
var_yaw.set(0)
Lya = Label(frameRJy, text="  Yaw:").pack(side=LEFT, fill=X, padx=10, pady=10)
But6p = Button(frameRJy, text="+", command=lambda: [var_yaw.set(var_yaw.get() + float(var_OrIncrement.get())), sendRaw()]).pack(side=LEFT)
But6n = Button(frameRJy, text="-", command=lambda: [var_yaw.set(var_yaw.get() - float(var_OrIncrement.get())), sendRaw()]).pack(side=LEFT)

# Create Interval of Change Spinbox for Rotational Jogging (Operations are same as Translational Spinbox)
frameRJsb = Frame(frameRJ)
frameRJsb.pack(side=TOP)
var_OrIncrement.set("1.00")
OrIncrement = Spinbox(frameRJsb, textvariable=var_OrIncrement, from_=0, to=10, justify=RIGHT, width=5, format='%.2f')
OrIncrement.pack(side=TOP, padx=10, pady=5)
LOrInc = Label(frameRJsb, text="Orientation Increment\n[deg]").pack(side=TOP, padx=10)


# Create Input Variable Frame under the Left Lower Frame
frameInputVariable = Frame(frameLEFTLOWER)
frameInputVariable.pack(side=TOP)

# Create Parameters Frame under the Left Lower Frame
frameParameters = Frame(frameLEFTLOWER)
frameParameters.pack(side=TOP)

# Create Frame for buttons under the Left Lower Frame
frameButtons = Frame(frameLEFTLOWER)
frameButtons.pack(side=TOP, pady=10)

# Create Left Side of The Inputs
frameL1L = Frame(frameInputVariable)
frameL1L.pack(side=LEFT)

# Create Right Side of The Inputs
frameL1R = Frame(frameInputVariable)
frameL1R.pack(side=LEFT)

# Create X Axis Input
frameL1Lx = Frame(frameL1L)
frameL1Lx.pack(side=TOP)
# Its Label
CTxLabel = Label(frameL1Lx, text='X:').pack(side=LEFT, fill=X, padx=10, pady=10)
# Its Spinbox
cTx = Spinbox(frameL1Lx, textvariable=var_x, from_=MinRange(0, 0, 0, 0, 0, 0), to=MaxRange(0, 0, 0, 0, 0, 0), justify=RIGHT, width=7, format='%.2f')
cTx.pack(side=LEFT, padx=5, pady=5)

# Create Y Axis Input (Orientation is same as X Axis)
frameL1Ly = Frame(frameL1L)
frameL1Ly.pack(side=TOP)
CTyLabel = Label(frameL1Ly, text='Y:').pack(side=LEFT, fill=X, padx=10, pady=10)
cTy = Spinbox(frameL1Ly, textvariable=var_y, from_=MinRange(1, 0, 0, 0, 0, 0), to=MaxRange(1, 0, 0, 0, 0, 0), justify=RIGHT, width=7, format='%.2f')
cTy.pack(side=LEFT, padx=5, pady=5)

# Create Z Axis Input (Orientation is same as X Axis)
frameL1Lz = Frame(frameL1L)
frameL1Lz.pack(side=TOP)
CTzLabel = Label(frameL1Lz, text='Z:').pack(side=LEFT, fill=X, padx=10, pady=10)
cTz = Spinbox(frameL1Lz, textvariable=var_z, from_=MinRange(2, 0, 0, 0, 0, 0), to=MaxRange(2, 0, 0, 0, 0, 0), justify=RIGHT, width=7, format='%.2f')
cTz.pack(side=LEFT, padx=5, pady=5)

# Create Pitch Axis Input (Orientation is same as X Axis)
frameL1Rp = Frame(frameL1R)
frameL1Rp.pack(side=TOP)
CTpLabel = Label(frameL1Rp, text='Pitch:').pack(side=LEFT, fill=X, padx=10, pady=10)
cTpitch = Spinbox(frameL1Rp, textvariable=var_pitch, from_=MinRange(3, 0, 0, 0, 0, 0), to=MaxRange(3, 0, 0, 0, 0, 0), justify=RIGHT, width=7, format='%.2f')
cTpitch.pack(side=LEFT, padx=5, pady=5)

# Create Roll Axis Input (Orientation is same as X Axis)
frameL1Rr = Frame(frameL1R)
frameL1Rr.pack(side=TOP)
CTrLabel = Label(frameL1Rr, text=' Roll:').pack(side=LEFT, fill=X, padx=10, pady=10)
cTroll = Spinbox(frameL1Rr, textvariable=var_roll, from_=MinRange(4, 0, 0, 0, 0, 0), to=MaxRange(4, 0, 0, 0, 0, 0), justify=RIGHT, width=7, format='%.2f')
cTroll.pack(side=LEFT, padx=5, pady=5)

# Create Yaw Axis Input (Orientation is same as X Axis)
frameL1Ry = Frame(frameL1R)
frameL1Ry.pack(side=TOP)
CTyaLabel = Label(frameL1Ry, text='  Yaw:').pack(side=LEFT, fill=X, padx=10, pady=10)
cTyaw = Spinbox(frameL1Ry, textvariable=var_yaw, from_=MinRange(5, 0, 0, 0, 0, 0), to=MaxRange(5, 0, 0, 0, 0, 0), justify=RIGHT, width=7, format='%.2f')
cTyaw.pack(side=LEFT, padx=5, pady=5)

# Create Spinbox for StepSize
StepSizeLab = Label(frameParameters, text='Step\nSize:').pack(side=LEFT, fill=X, padx=10, pady=10)
var_StepSize.set(20)
StepSizeSb = Spinbox(frameParameters, textvariable=var_StepSize, from_=0, to=3*var_Time.get(), justify=RIGHT, width=7)
StepSizeSb.pack(side=LEFT, padx=5, pady=5)

# Create Spinbox for Time
TimeLab = Label(frameParameters, text='Simulation\nTime [s]:').pack(side=LEFT, fill=X, padx=10, pady=10)
var_Time.set(10)
TimeSb = Spinbox(frameParameters, textvariable=var_Time, from_=0, justify=RIGHT, width=7)
TimeSb.pack(side=LEFT, padx=5, pady=5)

SButton = Button(frameButtons, text='Start', command=lambda: SetStateOn())
SButton.pack(side=TOP)

# Create Right Frame
frameRIGHT = Frame(root)
frameRIGHT.pack(side=LEFT)

# Create Right Upper Frame
frameIMU = Frame(frameRIGHT)
frameIMU.pack(side=TOP)

# Add Heading to Right Upper Frame
HeadingIMU = Label(frameIMU, text="IMU Data", font='Helvetica 18 bold').pack(side=TOP, fill=X, padx=10, pady=10)

# Create Right Upper Frame
frameRIGHTUPPER = Frame(frameRIGHT)
frameRIGHTUPPER.pack(side=TOP)

# Add Heading to Right Upper Frame
HeadingRIGHTUPPER = Label(frameRIGHTUPPER, text="End Effector", font='Helvetica 18 bold').pack(side=TOP, fill=X, padx=10, pady=10)

# Create Right Lower Frame
frameRIGHTLOWER = Frame(frameRIGHT)
frameRIGHTLOWER.pack(side=TOP)

# Add Heading to Right Lower Frame
HeadingRIGHTLOWER = Label(frameRIGHTLOWER, text="RC Servo Motor Angles", font='Helvetica 18 bold').pack(side=TOP, fill=X, padx=10, pady=10)

# Two Divisons of Right Upper Frame
frameIMU_L = Frame(frameIMU)
frameIMU_L.pack(side=LEFT)

frameIMU_R = Frame(frameIMU)
frameIMU_R.pack(side=LEFT)

# Create X on Left Divison
frameIMU_L_x = Frame(frameIMU_L)
frameIMU_L_x.pack(side=TOP)
var_IMU_x_acc.set('0.0')
# Add Label
LabelIMUX = Label(frameIMU_L_x, text=('X Acc: ' + var_IMU_x_acc.get() + ' mm'))
LabelIMUX.pack(side=LEFT, fill=X, padx=10, pady=10)

# Create Y on Left Divison (Operations are same as X)
frameIMU_L_y = Frame(frameIMU_L)
frameIMU_L_y.pack(side=TOP)
var_IMU_y_acc.set('0.0')
LabelIMUY = Label(frameIMU_L_y, text=('Y Acc: ' + var_IMU_y_acc.get() + ' mm'))
LabelIMUY.pack(side=LEFT, fill=X, padx=10, pady=10)

# Create Z on Left Divison (Operations are same as X)
frameIMU_L_z = Frame(frameIMU_L)
frameIMU_L_z.pack(side=TOP)
var_IMU_z_acc.set('0.0')
LabelIMUZ = Label(frameIMU_L_z, text=('Z Acc: ' + var_IMU_z_acc.get() + ' mm'))
LabelIMUZ.pack(side=LEFT, fill=X, padx=10, pady=10)

# Create Pitch on Right Divison (Operations are same as X)
frameIMU_R_p = Frame(frameIMU_R)
frameIMU_R_p.pack(side=TOP)
var_IMU_pitch.set('0.0')
LabelIMUP = Label(frameIMU_R_p, text=('Pitch: ' + var_IMU_pitch.get() + ' deg'))
LabelIMUP.pack(side=LEFT, fill=X, padx=10, pady=10)

# Create Roll on Right Divison (Operations are same as X)
frameIMU_R_r = Frame(frameIMU_R)
frameIMU_R_r.pack(side=TOP)
var_IMU_roll.set('0.0')
LabelIMUR = Label(frameIMU_R_r, text=(' Roll: ' + var_IMU_roll.get() + ' deg'))
LabelIMUR.pack(side=LEFT, fill=X, padx=10, pady=10)

# Create Yaw on Right Divison (Operations are same as X)
frameIMU_R_ya = Frame(frameIMU_R)
frameIMU_R_ya.pack(side=TOP)
var_IMU_yaw.set('0.0')
LabelIMUYa = Label(frameIMU_R_ya, text=('  Yaw: ' + var_IMU_yaw.get() + ' deg'))
LabelIMUYa.pack(side=LEFT, fill=X, padx=10, pady=10)


# Two Divisons of Right Upper Frame
frameRU_Left = Frame(frameRIGHTUPPER)
frameRU_Left.pack(side=LEFT)

frameRU_Right = Frame(frameRIGHTUPPER)
frameRU_Right.pack(side=LEFT)

# Two Divisions of Right Lower Frame
frameRL_Upper = Frame(frameRIGHTLOWER)
frameRL_Upper.pack(side=TOP)

frameRL_Lower = Frame(frameRIGHTLOWER)
frameRL_Lower.pack(side=TOP)

# Create X on Left Divison
frameRU_Left_x = Frame(frameRU_Left)
frameRU_Left_x.pack(side=TOP)
# Add Label
LabelX = Label(frameRU_Left_x, text=('X: ' + str(var_x.get()) + ' mm'))
LabelX.pack(side=LEFT, fill=X, padx=10, pady=10)

# Create Y on Left Divison (Operations are same as X)
frameRU_Left_y = Frame(frameRU_Left)
frameRU_Left_y.pack(side=TOP)
LabelY = Label(frameRU_Left_y, text=('Y: ' + str(var_y.get()) + ' mm'))
LabelY.pack(side=LEFT, fill=X, padx=10, pady=10)

# Create Z on Left Divison (Operations are same as X)
frameRU_Left_z = Frame(frameRU_Left)
frameRU_Left_z.pack(side=TOP)
LabelZ = Label(frameRU_Left_z, text=('Z: ' + str(var_z.get()) + ' mm'))
LabelZ.pack(side=LEFT, fill=X, padx=10, pady=10)

# Create Pitch on Right Divison (Operations are same as X)
frameRU_Right_p = Frame(frameRU_Right)
frameRU_Right_p.pack(side=TOP)
LabelP = Label(frameRU_Right_p, text=('Pitch: ' + str(var_pitch.get()) + ' deg'))
LabelP.pack(side=LEFT, fill=X, padx=10, pady=10)

# Create Roll on Right Divison (Operations are same as X)
frameRU_Right_r = Frame(frameRU_Right)
frameRU_Right_r.pack(side=TOP)
LabelR = Label(frameRU_Right_r, text=(' Roll: ' + str(var_roll.get()) + ' deg'))
LabelR.pack(side=LEFT, fill=X, padx=10, pady=10)

# Create Yaw on Right Divison (Operations are same as X)
frameRU_Right_y = Frame(frameRU_Right)
frameRU_Right_y.pack(side=TOP)
LabelYa = Label(frameRU_Right_y, text=('  Yaw: ' + str(var_yaw.get()) + ' deg'))
LabelYa.pack(side=LEFT, fill=X, padx=10, pady=10)

# Create 1st Servo Motor Representation
frameServo_1 = Frame(frameRL_Upper)
frameServo_1.pack(side=LEFT)
# Define Servo Motor Canvas
servo1 = Canvas(frameServo_1, width=100, height=100)
# Create a circle
circle1 = servo1.create_oval(10, 10, 100, 100)
# Create a line with arrow which shows initial position of servo motor
line1 = servo1.create_line(centerServo[0], centerServo[1], centerServo[0]-radiusServo*cos(0), centerServo[1]-radiusServo*sin(0), arrow=LAST)
# Create text to show angle value
text1 = servo1.create_text(centerServo, text='{0:.1f}'.format(180))
# Box of text
box1 = servo1.create_rectangle(servo1.bbox(text1), fill='white', outline='white', width=5)
servo1.tag_lower(box1, text1)
servo1.pack(side=TOP)
# Label of Servo
Ls1 = Label(frameServo_1, text="Servo 1").pack(side=TOP, fill=X)

# Create 2nd Servo Motor Representation (Operations are same as 1st Servo)
frameServo_2 = Frame(frameRL_Upper)
frameServo_2.pack(side=LEFT)
servo2 = Canvas(frameServo_2, width=100, height=100)
circle2 = servo2.create_oval(10, 10, 100, 100)
line2 = servo2.create_line(centerServo[0], centerServo[1], centerServo[0]+radiusServo*cos(0), centerServo[1]+radiusServo*sin(0), arrow=LAST)
text2 = servo2.create_text(centerServo, text='{0:.1f}'.format(0))
box2 = servo2.create_rectangle(servo2.bbox(text1), fill='white', outline='white', width=5)
servo2.tag_lower(box2, text2)
servo2.pack(side=TOP)
Ls2 = Label(frameServo_2, text="Servo 2").pack(side=TOP, fill=X)

# Create 3rd Servo Motor Representation (Operations are same as 1st Servo)
frameServo_3 = Frame(frameRL_Upper)
frameServo_3.pack(side=LEFT)
servo3 = Canvas(frameServo_3, width=100, height=100)
circle3 = servo3.create_oval(10, 10, 100, 100)
line3 = servo3.create_line(centerServo[0], centerServo[1], centerServo[0]-radiusServo*cos(0), centerServo[1]-radiusServo*sin(0), arrow=LAST)
text3 = servo3.create_text(centerServo, text='{0:.1f}'.format(180))
box3 = servo3.create_rectangle(servo3.bbox(text1), fill='white', outline='white', width=5)
servo3.tag_lower(box3, text3)
servo3.pack(side=TOP)
Ls3 = Label(frameServo_3, text="Servo 3").pack(side=TOP, fill=X)

# Create 4th Servo Motor Representation (Operations are same as 1st Servo)
frameServo_4 = Frame(frameRL_Lower)
frameServo_4.pack(side=LEFT)
servo4 = Canvas(frameServo_4, width=100, height=100)
circle4 = servo4.create_oval(10, 10, 100, 100)
line4 = servo4.create_line(centerServo[0], centerServo[1], centerServo[0]+radiusServo*cos(0), centerServo[1]+radiusServo*sin(0), arrow=LAST)
text4 = servo4.create_text(centerServo, text='{0:.1f}'.format(0))
box4 = servo4.create_rectangle(servo4.bbox(text1), fill='white', outline='white', width=5)
servo4.tag_lower(box4, text4)
servo4.pack(side=TOP)
Ls4 = Label(frameServo_4, text="Servo 4").pack(side=TOP, fill=X)

# Create 5th Servo Motor Representation (Operations are same as 1st Servo)
frameServo_5 = Frame(frameRL_Lower)
frameServo_5.pack(side=LEFT)
servo5 = Canvas(frameServo_5, width=100, height=100)
circle5 = servo5.create_oval(10, 10, 100, 100)
line5 = servo5.create_line(centerServo[0], centerServo[1], centerServo[0]-radiusServo*cos(0), centerServo[1]-radiusServo*sin(0), arrow=LAST)
text5 = servo5.create_text(centerServo, text='{0:.1f}'.format(180))
box5 = servo5.create_rectangle(servo5.bbox(text1), fill='white', outline='white', width=5)
servo5.tag_lower(box5, text5)
servo5.pack(side=TOP)
Ls5 = Label(frameServo_5, text="Servo 5").pack(side=TOP, fill=X)

# Create 6th Servo Motor Representation (Operations are same as 1st Servo)
frameServo_6 = Frame(frameRL_Lower)
frameServo_6.pack(side=LEFT)
servo6 = Canvas(frameServo_6, width=100, height=100)
circle6 = servo6.create_oval(10, 10, 100, 100)
line6 = servo6.create_line(centerServo[0], centerServo[1], centerServo[0]+radiusServo*cos(0), centerServo[1]+radiusServo*sin(0), arrow=LAST)
text6 = servo6.create_text(centerServo, text='{0:.1f}'.format(0))
box6 = servo6.create_rectangle(servo6.bbox(text1), fill='white', outline='white', width=5)
servo6.tag_lower(box6, text6)
servo6.pack(side=TOP)
Ls6 = Label(frameServo_6, text="Servo 6").pack(side=TOP, fill=X)


# Loop Over the GUI
root.mainloop()

