# -*- coding: utf-8 -*-


# import libraries
import numpy as np
import serial
import serial.tools.list_ports as port_list
import time
import tkinter as tk
import tkinter.ttk as ttk


ports = list(port_list.comports())  # get list of ports

for n in range(len(ports)):  # find the ports and assign them 
    x = str(ports[n]).find('Arduino')  # identify arduino from port list
    y = str(ports[n].hwid).find('VID:PID=3604:00F1')  # identify monochromator from hardware data
    z = str(ports[n].hwid).find('VID:PID=3604:00E1')  # identify detector from hardware data
    if x != -1:
        arduinoport = ports[n].device  # assign as the port connected to the arduino
    if y != -1:
        monochromatorport = ports[n].device  # assign as the port connected to the monochromator
    if z != -1:
        detectorport = ports[n].device  # assign as the port connected to the detector 

arduino_port = serial.Serial(port = arduinoport, baudrate = 115200, bytesize = 8, 
                             timeout = 1, stopbits = serial.STOPBITS_ONE, rtscts = True, 
                             parity = serial.PARITY_NONE)  # configure port for use with arduino controller
arduino_port.flush()  # flush serial buffer

monochromator_port = serial.Serial(port = monochromatorport, baudrate = 9600, bytesize = 8, 
                                   timeout = 2, stopbits = serial.STOPBITS_ONE, rtscts = True, 
                                   parity = serial.PARITY_NONE)  # configure port for use with monochromator
monochromator_port.flush()  # flush serial buffer

detector_port = serial.Serial(port = detectorport, baudrate = 115200, bytesize = 8,
                              timeout = 1, stopbits = serial.STOPBITS_ONE, rtscts = True,
                              parity = serial.PARITY_NONE)  # configure port for use with monochromator
detector_port.flush()  # flush serial buffer

time.sleep(2)  # allow items connected to ports to wake up


#  initialising motor positions
polariser_position = 0.0
analyser_position = 0.0
theta_reflected_position = 0.0
theta_incident_position = 0.0
height_position = 0.0
xtilt_position = 0.0
ytilt_position = 0.0

# defining arrays for spectrometer data
wavelength_array = []
spectrum_array = []

# initialising counter for stage jog
height_jog = 0.0
xtilt_jog = 0.0
ytilt_jog = 0.0


def onclose():
    ''' 
    Tidies on program close
    '''
    homeIR()  # drive both arms to their home position
    arduino_port.close()  # close arduino port
    monochromator_port.close()  # close monochromator port
    detector_port.close()  # close detector port
    time.sleep(1)
    window.destroy()  # close the window   

    
def arduino_comms(command):
    ''' 
    Handles all communication with the arduino 
    '''
    arduino_port.reset_input_buffer()  # clear previous entries to serial port
    
    input_command = command+'\n'  # build command
    arduino_port.write(input_command.encode("utf8"))  # send command to arduino
    
    while (arduino_port.inWaiting()==0):  # for a new entry to appear in the serial port
        pass
    
    incoming_string = arduino_port.readline().decode().strip()  # read new serial port entry
    print("Arduino response:", incoming_string)  # print arduino response for testing purposes
    
    return incoming_string


def moveTI():
    ''' 
    Moves the angle of incidence arm
    '''
    global theta_incident_position
    
    move = int(theta_incident_entry.get()) - 90  # obtain input angle and convert relative to 90 degree home position
    position = arduino_comms('MOVEI '+str(move))  # send move command and recieves current position
    theta_incident_position = float(position) + 90  # convert new position to angle of incidence and save to variable
    
    return theta_incident_position
    
    
def moveTR():
    ''' 
    Moves the angle of reflection arm 
    '''
    global theta_reflected_position
    
    move = int(theta_reflected_entry.get()) - 90  # obtain input angle and convert relative to 90 degree home position
    position = arduino_comms('MOVER '+str(move))  # send move command and recieve current position
    theta_reflected_position = float(position) + 90 # convert new position to angle of incidence and save to variable
    
    return theta_reflected_position
   
    
def moveTIR():
    ''' 
    Moves both arms simultaneously 
    '''
    global theta_incident_position, theta_reflected_position
    
    move = int(theta_start_entry.get()) - 90  # obtain input angle and convert relative to 90 degree home position
    position = arduino_comms('MOVEIR '+str(move)+','+str(move))  # send move command and recieve current position
    
    index = position.index(',')
    positionI = position[0: index]  # obtain position of incident arm
    positionR = position[index+1:]  # obtain position of reflected arm 
    
    theta_incident_position = float(positionI) + 90  # convert new position to angle of incidence and save to variable
    theta_reflected_position = float(positionR) + 90  # convert new position to angle of incidence and save to variable
    
    return theta_incident_position, theta_reflected_position


def moveP():
    ''' 
    Moves the polariser 
    '''
    global polariser_position 
    
    position = arduino_comms('MOVEP '+polariser_entry.get())  # send move command and recieve current position
    polariser_position = float(position)  # save new position to variable
    
    return polariser_position
 
   
def moveA():
    ''' 
    Moves the analyser 
    '''
    global analyser_position
    
    position = arduino_comms('MOVEA '+analyser_entry.get())  # send move command and recieve current position
    analyser_position = float(position)  # save new position to variable
    
    return analyser_position
    
    
def moveH_pos():
    ''' 
    Moves the height stage in the positive direction slow
    '''
    global height_position, height_jog
    
    height_jog += 1  # running counter for jog button in degrees - CHANGE SPEED HERE
    position = arduino_comms('MOVEH '+str(height_jog))  # send move command and recieve current position
    height_position = float(position)  # save new position to variable
    
    return height_position


def moveH_posplus():
    ''' 
    Moves the height stage in the positive direction fast
    '''
    global height_position, height_jog
    
    height_jog += 10  # running counter for jog button in degrees - CHANGE SPEED HERE
    position = arduino_comms('MOVEH '+str(height_jog))  # send move command and recieve current position
    height_position = float(position)  # save new position to variable
    
    return height_position

    
def moveH_neg():
    ''' 
    Moves the height stage in the negative direction slow
    '''
    global height_position, height_jog
    
    height_jog -= 1  # running counter for jog button in degrees - CHANGE SPEED HERE
    position = arduino_comms('MOVEH '+str(height_jog))  # send move command and recieve current position
    height_position = float(position)  # save new position to variable 
    
    return height_position


def moveH_negplus():
    ''' 
    Moves the height stage in the negative direction fast
    '''
    global height_position, height_jog
    
    height_jog -= 10  # running counter for jog button in degrees - CHANGE SPEED HERE
    position = arduino_comms('MOVEH '+str(height_jog))  # send move command and recieve current position
    height_position = float(position)  # save new position to variable
    
    return height_position
    
    
def moveX_pos():
    ''' 
    Adjusts the x-tilt in the positive direction slow
    '''
    global xtilt_position, xtilt_jog
    
    xtilt_jog += 1  # running counter for jog button in degrees - CHANGE SPEED HERE
    position = arduino_comms('MOVEX '+str(xtilt_jog))  # send move command and recieve current position
    xtilt_position = float(position)  # save new position to variable
    
    return xtilt_position
    

def moveX_posplus():
    ''' 
    Adjusts the x-tilt in the positive direction fast
    '''
    global xtilt_position, xtilt_jog
    
    xtilt_jog += 5  # running counter for jog button in degrees - CHANGE SPEED HERE
    position = arduino_comms('MOVEX '+str(xtilt_jog))  # send move command and recieve current position
    xtilt_position = float(position)  # save new position to variable
    
    return xtilt_position
  
  
def moveX_neg():
    ''' 
    Adjusts the x-tilt in the negative direction slow
    '''
    global xtilt_position, xtilt_jog
    
    xtilt_jog -= 1  # running counter for jog button in degrees - CHANGE SPEED HERE
    position = arduino_comms('MOVEX '+str(xtilt_jog))  # send move command and recieve current position
    xtilt_position = float(position)  # save new position to variable
    
    return xtilt_position  
  
  
def moveX_negplus():
    ''' 
    Adjusts the x-tilt in the negative direction fast
    '''
    global xtilt_position, xtilt_jog
    
    xtilt_jog -= 5  # running counter for jog button in degrees - CHANGE SPEED HERE
    position = arduino_comms('MOVEX '+str(xtilt_jog))  # send move command and recieve current position
    xtilt_position = float(position)  # save new position to variable
    
    return xtilt_position
 
   
def moveY_pos():
    ''' 
    Adjusts the y-tilt in the positive direction slow
    '''
    global ytilt_position, ytilt_jog
    
    ytilt_jog += 1  # running counter for jog button in degrees - CHANGE SPEED HERE
    position = arduino_comms('MOVEY '+str(ytilt_jog))  # send move command and recieve current position
    ytilt_position = float(position)  # save new position to variable
    
    return ytilt_position


def moveY_posplus():
    ''' 
    Adjusts the y-tilt in the positive direction fast
    '''
    global ytilt_position, ytilt_jog
    
    ytilt_jog += 5  # running counter for jog button in degrees - CHANGE SPEED HERE
    position = arduino_comms('MOVEY '+str(ytilt_jog))  # send move command and recieve current position
    ytilt_position = float(position)  # save new position to variable
    
    return ytilt_position


def moveY_neg():
    ''' 
    Adjusts the y-tilt in the negative direction slow
    '''
    global ytilt_position, ytilt_jog
    
    ytilt_jog -= 1  # running counter for jog button in degrees - CHANGE SPEED HERE
    position = arduino_comms('MOVEY '+str(ytilt_jog))  # send move command and recieve current position
    ytilt_position = float(position)  # save new position to variable
    
    return ytilt_position


def moveY_negplus():
    ''' 
    Adjusts the y-tilt in the negative direction fast
    '''
    global ytilt_position, ytilt_jog
    
    ytilt_jog -= 5  # running counter for jog button in degrees - CHANGE SPEED HERE
    position = arduino_comms('MOVEY '+str(ytilt_jog))  # send move command and recieve current position
    ytilt_position = float(position)  # save new position to variable
    
    return ytilt_position


def homeTI():
    ''' 
    Moves the angle of incidence arm to the home position 
    '''
    global theta_incident_position
    
    position = arduino_comms('HOMEI ')  # send home command and recieve current position
    theta_incident_position = float(position) + 90  # convert new position to angle of incidence and save to variable
    
    return theta_incident_position
    
    
def homeTR():
    ''' 
    Moves the angle of reflection arm to the home position 
    '''
    global theta_reflected_position
    
    position = arduino_comms('HOMER ')  # send home command and recieve current position
    theta_reflected_position = float(position) + 90  # convert new position to angle of incidence and save to variable
    
    return theta_reflected_position


def homeIR():
    '''
    Moves both arms to the home position 
    '''
    global theta_incident_position, theta_reflected_position
    
    position = arduino_comms('HOMIR ')  # send home command and recieve current position 
    
    index = position.index(',')
    positionI = position[0: index]  # obtain position of incident arm
    positionR = position[index+1:]  # obtain position of reflected arm
    
    theta_incident_position = float(positionI) + 90  # convert new position to angle of incidence and save to variable
    theta_reflected_position = float(positionR) + 90  # convert new position to angle of incidence and save to variable
    
    return theta_incident_position, theta_reflected_position


def get_spectrum(wavelength_start, wavelength_stop, wavelength_step, spectrum_size):
    ''' 
    Obtains a single spectrum over a specified wavelength range 
    '''
    
    spectrum = np.zeros(spectrum_size)  # initialise array to hold intensity values
    
    for i in np.arange(wavelength_start, wavelength_stop+1, wavelength_step):  # loop to cycle through wavelengths
        
        wavelength_comms = ('<wavelengthd '+str(int(i))+'>')  # build the command
        monochromator_port.write(wavelength_comms.encode())  # send command to the monochromator
        monochromator_port.write('<wavelength?>'.encode())  
        
        detector_port.write('<read>'.encode("utf8"))  # request intensity reading from detector
        intensity = detector_port.readline().decode()  # save intensity reading
        
        spectrum[i] = float(intensity)  # append intensity reading to array
        
    return spectrum
        
          
def sweep():
    ''' 
    Takes a spectrum for each polariser step for each arm angle 
    '''
    
    progress_bar.set(0)  # reset progress bar
    
    theta_start = float(theta_start_entry.get())  # get arms start angle from entry box
    theta_stop = float(theta_stop_entry.get())  # get arms stop angle from entry box
    theta_step = float(theta_step_entry.get())  # get arms angle steps from entry box
    polariser_step = float(polariser_step_entry.get())  # get polariser angle steps from entry box
    wavelength_start = float(wavelength_start_entry.get())  # get monochromator start wavelength from entry box
    wavelength_stop = float(wavelength_stop_entry.get())  # get monochomator stop wavelength from entry box
    wavelength_step = float(wavelength_step_entry.get())  # get monochromator wavelength steps from entry box
    
    n = abs((theta_start - theta_stop) / theta_step) + 1  # calculate how many steps the arms must take 
    m = 360 / polariser_step  # calculate how many steps polariser must take
    
    arms_moveto = theta_start - 90  # initial position of arms 
    polariser_moveto = 0  # initial position of polariser 
    
    progress_size = 99 / (n * m)  # calculate the size of each progress bar update
    wavelength_size = round((wavelength_stop - wavelength_start) / wavelength_step)  # calculate the number of entries in the wavelength array
    
    wavelength = np.linspace(wavelength_start, wavelength_stop, wavelength_size)  # array containing all wavelengths
    polariser_angles = np.linspace(0, 360, m)  # array containing all polariser angles 
    angles_incidence = np.linspace(theta_start, theta_stop, n)  # array containing all angles of incidence
    
    for i in np.arange(0, n): # loop to step arms simultaneously
    
        arduino_comms('MOVIR '+str(arms_moveto))  # move arms simultaneously
        
        if theta_start < theta_stop:  # if angle of incidence must increase
            arms_moveto += theta_step  # step arms position positively
        else:  # otherwise
            arms_moveto -= theta_step  # step arms position negatively 
        
        for j in np.arange(0, m):  # loop to step polariser 
            
            arduino_comms('MOVEP '+str(polariser_moveto))  # move polariser
            
            spectrum = get_spectrum(wavelength_start, wavelength_stop, wavelength_step, wavelength_size)  # obtain spectrum for current position
            
            polariser_moveto += polariser_step  # step polariser position 
            
            progress_bar.step(progress_size)  # step progress bar
            progress_bar.update()  # update progress bar
             


# ------ Tkinter GUI ------


class LatchButton(tk.Button):
    '''
    Button that calls the command repeatedly when held down 
    '''
    def __init__(self, master = None, **kwargs):
        self.command = kwargs.pop('command', None)
        self.timeout = kwargs.pop('timeout', None)
        tk.Button.__init__(self, master, **kwargs)
        self.bind('<ButtonPress-1>', self.start)
        self.bind('<ButtonRelease-1>', self.stop)
        self.timer = ''

    def start(self, event = None):
        if self.command is not None:
            self.command()
            if self.timeout is not None:
                self.timer = self.after(self.timeout, self.start)

    def stop(self, event = None):
        self.after_cancel(self.timer)


window = tk.Tk()
window.geometry("800x300")
window.title("Ellipsometer Control Software")
window.configure(background = 'DodgerBlue4')

notestyle = ttk.Style()
notestyle.configure("TNotebook.Tab", background = 'DodgerBlue4')
tabControl = ttk.Notebook(window)

utilities = tk.Frame(tabControl, background = 'white')
measurement = tk.Frame(tabControl, background = 'white')
analysis = tk.Frame(tabControl, background = 'white')

tabControl.add(utilities, text = 'Utilities')
tabControl.add(measurement, text = 'Measurement')
tabControl.add(analysis, text = 'Analysis')

tabControl.pack(fill = "x")


# measurement widgets
wavelength_start_entry_label = tk.Label(measurement, text = "Wavelength Start", 
                                        background='white').grid(row = 1, column = 1)
wavelength_start_entry = ttk.Entry(measurement, width = 10)
wavelength_start_entry.insert(0,"0")
wavelength_start_entry.grid(column = 2, row = 1, padx = 10, pady = 5)


wavelength_stop_entry_label = tk.Label(measurement, text = "Wavelength Stop", 
                                       background='white').grid(row = 2, column = 1)
wavelength_stop_entry = ttk.Entry(measurement, width = 10)
wavelength_stop_entry.insert(0,"0")
wavelength_stop_entry.grid(column = 2, row = 2, padx = 10, pady = 5)


wavelength_step_entry_label = tk.Label(measurement, text = "Wavelength Step", 
                                       background='white').grid(row = 3, column = 1)
wavelength_step_entry = ttk.Entry(measurement, width = 10)
wavelength_step_entry.insert(0,"0")
wavelength_step_entry.grid(column = 2, row = 3, padx = 10, pady = 5)


theta_start_entry_label = tk.Label(measurement, text = "Theta Start", 
                                   background='white').grid(row = 1, column = 3)
theta_start_entry = ttk.Entry(measurement, width = 10)
theta_start_entry.insert(0,"0")
theta_start_entry.grid(column = 4, row = 1, padx = 10, pady = 5)


theta_stop_entry_label = tk.Label(measurement, text = "Theta Stop", 
                                  background='white').grid(row = 2, column = 3)
theta_stop_entry = ttk.Entry(measurement, width = 10)
theta_stop_entry.insert(0,"0")
theta_stop_entry.grid(column = 4, row = 2, padx = 10, pady = 5)


theta_step_entry_label = tk.Label(measurement, text = "Theta Step", 
                                  background='white').grid(row = 3, column = 3)
theta_step_entry = ttk.Entry(measurement, width = 10)
theta_step_entry.insert(0,"0")
theta_step_entry.grid(column = 4, row = 3, padx = 10, pady = 5)


polariser_step_entry_label = tk.Label(measurement, text = "Polariser Step", 
                                      background='white').grid(row = 1, column = 5)
polariser_step_entry = ttk.Entry(measurement, width = 10)
polariser_step_entry.insert(0,"0")
polariser_step_entry.grid(column = 6, row = 1, padx = 10, pady = 5)


sweep_button = tk.Button(measurement, text = "Begin Sweep", command = sweep, 
                         background = 'white')
sweep_button.grid(column = 2, row = 4, padx = 0, pady = 50)


progress_bar = ttk.Progressbar(measurement, maximum = 99.1)
progress_bar.grid(row = 4, column = 4, columnspan = 3, padx = 0, pady = 0, 
                 sticky = 'ew')


# utilities widgets
theta_incident_entry_label = tk.Label(utilities, text = "Theta Incident (Degs)", 
                                      background = 'white').grid(row = 1, column = 1)
theta_incident_entry = ttk.Entry(utilities, width = 10)
theta_incident_entry.insert(0,"90")
theta_incident_entry.grid(column = 2, row = 1, padx = 10, pady = 5)
theta_incident_move_button = tk.Button(utilities, text = "Move", command = moveTI, 
                                       background = 'white') 
theta_incident_move_button.grid(column = 3, row = 1, padx = 20, pady = 5)


theta_reflected_entry_label = tk.Label(utilities, text = "Theta Reflected (Degs)", 
                                       background = 'white').grid(row = 2, column = 1)
theta_reflected_entry = ttk.Entry(utilities, width = 10)
theta_reflected_entry.insert(0,"90")
theta_reflected_entry.grid(column = 2, row = 2, padx = 10, pady = 5)
theta_reflected_move_button = tk.Button(utilities, text = "Move", command = moveTR, 
                                        background = 'white') 
theta_reflected_move_button.grid(column = 3, row = 2, padx = 20, pady = 5)


polariser_entry_label = tk.Label(utilities, text = "Polariser (Degs)", 
                                 background = 'white').grid(row = 3,column = 1)
polariser_entry = ttk.Entry(utilities, width = 10)
polariser_entry.insert(0,"0")
polariser_entry.grid(column = 2, row = 3, padx = 10, pady = 5)
polariser_move_button = tk.Button(utilities, text = "Move", command = moveP, 
                                  background = 'white') 
polariser_move_button.grid(column = 3, row = 3, padx = 20, pady = 5)


analyser_entry_label = tk.Label(utilities, text = "Analyser (Degs)", 
                                background = 'white').grid(row = 4,column = 1)
analyser_entry = ttk.Entry(utilities, width = 10)
analyser_entry.insert(0,"0")
analyser_entry.grid(column = 2, row = 4, padx = 10, pady = 5)
analyser_move_button = tk.Button(utilities, text = "Move", command = moveA, 
                                 background = 'white') 
analyser_move_button.grid(column = 3, row = 4, padx = 20, pady = 5)


height_buttons_label = tk.Label(utilities, text = "Stage Height (mm)", 
                                background = 'white').grid(row = 1,column = 5)
height_button_pos = LatchButton(utilities, command = moveH_pos, 
                                timeout = 3, text = '+')
height_button_pos.grid(column = 5, row = 3, padx = 0, pady = 5)
height_button_neg = LatchButton(utilities, command = moveH_neg, 
                                timeout = 3, text = '-')
height_button_neg.grid(column = 5, row = 4, padx = 0, pady = 5)
height_button_posplus = LatchButton(utilities, command = moveH_posplus, 
                                    timeout = 3, text = '++')
height_button_posplus.grid(column = 5, row = 2, padx = 0, pady = 5)
height_button_negplus = LatchButton(utilities, command = moveH_negplus, 
                                    timeout = 3, text = '--')
height_button_negplus.grid(column = 5, row = 5, padx = 0, pady = 5)


xtilt_buttons_label = tk.Label(utilities, text = "X-Tilt (mm)", 
                               background = 'white').grid(row = 1,column = 6)
xtilt_button_pos = LatchButton(utilities, command = moveX_pos, 
                               timeout = 3, text = '+')
xtilt_button_pos.grid(column = 6, row = 3, padx = 0, pady = 5)
xtilt_button_neg = LatchButton(utilities, command = moveX_neg, 
                               timeout = 3, text = '-')
xtilt_button_neg.grid(column = 6, row = 4, padx = 0, pady = 5)
xtilt_button_posplus = LatchButton(utilities, command = moveX_posplus, 
                                   timeout = 3, text = '++')
xtilt_button_posplus.grid(column = 6, row = 2, padx = 0, pady = 5)
xtilt_button_negplus = LatchButton(utilities, command = moveX_negplus, 
                                   timeout = 3, text = '--')
xtilt_button_negplus.grid(column = 6, row = 5, padx = 0, pady = 5)


ytilt_buttons_label = tk.Label(utilities, text = "Y-Tilt (mm)", 
                               background = 'white').grid(row = 1,column = 7)
ytilt_button_pos = LatchButton(utilities, command = moveY_pos, 
                               timeout = 3, text = '+')
ytilt_button_pos.grid(column = 7, row = 3, padx = 0, pady = 5)
ytilt_button_neg = LatchButton(utilities, command = moveY_neg, 
                               timeout = 3, text = '-')
ytilt_button_neg.grid(column = 7, row = 4, padx = 0, pady = 5)
ytilt_button_posplus = LatchButton(utilities, command = moveY_posplus, 
                                   timeout = 3, text = '++')
ytilt_button_posplus.grid(column = 7, row = 2, padx = 0, pady = 5)
ytilt_button_negplus = LatchButton(utilities, command = moveY_negplus, 
                                   timeout = 3, text = '--')
ytilt_button_negplus.grid(column = 7, row = 5, padx = 0, pady = 5)


theta_incident_home_button = tk.Button(utilities, text = "Home", command = homeTI, 
                                       background = 'white') 
theta_incident_home_button.grid(column = 4, row = 1, padx = 20, pady = 10)


theta_reflected_home_button = tk.Button(utilities, text = "Home", command = homeTR, 
                                        background = 'white') 
theta_reflected_home_button.grid(column = 4, row = 2, padx = 20, pady = 10)


window.protocol("WM_DELETE_WINDOW", onclose)
window.mainloop()

