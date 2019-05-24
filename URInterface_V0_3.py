import socket
#import threading
#import _thread
#from threading import Thread
import struct
import math
import time
import platform
#check python version
python_vrsn = platform.python_version()
#self.python_vrsn = "2"
print(python_vrsn)
if python_vrsn[0] == "2":
    #import threading
    import thread
elif python_vrsn[0] == "3":
    import _thread


class URController():

    """URController creates an interface between the UR and python applications through TCP/IP to control and retrieve information from the robot.

        The URController-class allows to move, set digital outputs and configure the robot through python script.
        It is also possible to retrieve information about position, speed and torques of the joints by getter-functions within
        the class. Reading data is either cyclic or acylic. Cyclic reading will open up a TCP/IP-connection and will periodically
        read datapackages send by the UR until the session is closed. Acyclic reading will only create and close a connection to
        read datapackages once.

        Attributes:
            PORT (int): is set to port 30003 using the real time interface of the UR.
            cycle(boolean): flag that is set to true if cyclic reading is turned on
    """
    # connection
    PORT = 30003

    # data
    data = [{"size": 4 , "value": None},        #  0 Message Size
            {"size": 8 , "value": None},        #  1 Time
            {"size": 48, "value": None},        #  2 q Target
            {"size": 48, "value": None},        #  3 qd Target
            {"size": 48, "value": None},        #  4 qdd target
            {"size": 48, "value": None},        #  5 I target
            {"size": 48, "value": None},        #  6 M target
            {"size": 48, "value": None},        #  7 q actual
            {"size": 48, "value": None},        #  8 qd actual
            {"size": 48, "value": None},        #  9 I actual
            {"size": 48, "value": None},        # 10 I control
            {"size": 48, "value": None},        # 11 Tool vector actual
            {"size": 48, "value": None},        # 12 TCP speed actual
            {"size": 48, "value": None},        # 13 TCP force
            {"size": 48, "value": None},        # 14 Tool vector target
            {"size": 48, "value": None},        # 15 TCP speed target
            {"size": 8 , "value": None},        # 16 Digital input bits
            {"size": 48, "value": None},        # 17 Motor
            {"size": 8 , "value": None},        # 18 Controller Timer
            {"size": 8 , "value": None},        # 19 Test value
            {"size": 8 , "value": None},        # 20 Robot Mode
            {"size": 48, "value": None},        # 21 Joint Mode
            {"size": 8 , "value": None},        # 22 Safety Mode
            {"size": 48, "value": None},        # 23 UR only
            {"size": 24, "value": None},        # 24 Tool
            {"size": 48, "value": None},        # 25 UR only
            {"size": 8 , "value": None},        # 26 Speed Scaling
            {"size": 8 , "value": None},        # 27 Linear momentum norm
            {"size": 8 , "value": None},        # 28 UR only 3
            {"size": 8 , "value": None},        # 29 UR only 4
            {"size": 8 , "value": None},        # 30 V main
            {"size": 8 , "value": None},        # 31 V robot
            {"size": 8 , "value": None},        # 32 I robot
            {"size": 48, "value": None},        # 33 V actual
            {"size": 8 , "value": None},        # 34 digital outputs
            {"size": 8 , "value": None}         # 35 program state
           ]

    def __init__(self, HOST_IP):

        """
        Args:
            HOST_IP (str): IP-address of the UR.
        """

        self.HOST_IP = HOST_IP

        #self.read_packages_acyclic()
        self.isReading = False
        self.cyclic = False
        

    def start_cyclic_reading_session(self):

        """ Starts a thread connecting to the UR and reading data packages periodically. """

        try:


            if self.cyclic is not True:

                # create connection
                self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.s.settimeout(10)
                self.s.connect((self.HOST_IP, self.PORT))

                self.cyclic = True

                #threading.Thread.__init__(self)
                self.run()
                



                print("Started cyclic reading session.")
            else:
                print("Session already started.")

        except socket.error as socketError:
            print(socketError)

    def run(self):
        
        if self.cyclic: 
                
                #print("read data package")
                self.isReading = True
                data_package = self.s.recv(3000)
                #print(len(data_package))
                #print(data_package)
                self.isReading = False
                counter = 0
                
                #print(len(data_package))
                # receive data for each package
                for d in self.data:
                    # save binary string in \x notation
                    d["value"] = data_package[counter: counter + d["size"]]
                    counter += d["size"]
                #self.cyclic = False
                #time.sleep(.005)
                

                try:
                    
                    if python_vrsn[0] == "2":
                        #threading.Thread.__init__(self)
                        thread.start_new_thread(self.run, ())
                        #self.start()
                    elif python_vrsn[0] == "3":
                        _thread.start_new_thread(self.run, ())
                
                except Exception:
                    import traceback
                    print(traceback.format_exc())
                


    def close_cyclic_reading_session(self):

        """Closing the reading cycle and the connection to the UR."""

        if self.cyclic:
            self.cyclic = False
            
            # close the connection
            print("Waiting for last reading...")
            while self.isReading == True:
                time.sleep(.5)
                pass
            self.s.close()

            # do not read data anymore
            

            print("Closed reading session.")

        else:

            print("No reading session started.")

# ------------------------------- READ DATA FUNCTIONS --------------------------------------------

    def read_packages_acyclic(self):
        print("Acyclic Reading")
        """Connects to the UR and reads datapackages once before closing the connection."""
        str_state = "Done"
        try:
      
            # create connection
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(10)
            s.connect((self.HOST_IP, self.PORT))
            # receive data for eack packages
            for d in self.data:
                # save binary string in \x notation
                d["value"] = s.recv(d["size"])

        except socket.error as socketError:
            print(socketError)
            str_state = socketError
        finally:
            # close the connection
            s.close()
            return str_state
        # ------------------------------- EXTRACT INFORMATION ABOUT UR -----------------------------------

# ------------------------------- GETTER FUNCTIONS -----------------------------------------------

    def get_time(self):

        """Returns time[ms] since the start of the UR-Controller."""

        if self.cyclic is not True:
            self.read_packages_acyclic()

        time = -1

        # get the time since the controller started
        if self.data[1]["value"] is not None:
            time = struct.unpack("!d" , self.data[1]["value"])

        return time

    def get_target_joint_coordinates(self):

        """Returns the six angles[rad] of each joint."""

        return self.get_list_of_six_values(2)

    def get_target_joint_velocities(self):

        """Returns the six angular velocities[rad/s] of each joint."""

        return self.get_list_of_six_values(3)

    def get_target_joint_accelerations(self):

        """Returns the six angular acclerations[rad/s^2] of each joint."""

        return self.get_list_of_six_values(4)

    def get_target_joint_currents(self):

        """Returns the six currents[A] of each joint."""

        return self.get_list_of_six_values(5)

    def get_target_joint_torques(self):

        """Returns the six torques[Nm] applied on each joint"""

        return self.get_list_of_six_values(6)

    def get_actual_joint_coordinates(self):

        """Returns the current angles[rad] of each joint."""

        return self.get_list_of_six_values(7)

    def get_actual_joint_velocities(self):

        """Returns the current angular velocities[rad/s] of each joint."""

        return self.get_list_of_six_values(8)

    def get_actual_joint_currents(self):

        """Returns the actual currents[A] of each joint."""

        return self.get_list_of_six_values(9)

    def get_joint_control_currents(self):

        """Returns the six control currents of each joint."""

        return self.get_list_of_six_values(10)

    def get_actual_tool_coordinates(self):

        """Returns the three cartesian coordinates[m] in tool-space and the three axis angles[rad]"""

        return self.get_list_of_six_values(11)

    def get_actual_tool_velocities(self):

        """Returns the three velocities[m/s] in x,y,z and the three angular velocities[rad/s]"""

        return self.get_list_of_six_values(12)

    def get_TCP_force(self):

        """Returns the three forces[N] and the three torques[Nm] in x,y,z acting on the TCP."""
        return self.get_list_of_six_values(13)

    def get_target_tool_coordinates(self):

        """Returns the desired coordinates[m & rad] of the tool."""

        return self.get_list_of_six_values(14)

    def get_target_TCP_speed(self):

        return self.get_list_of_six_values(15)

    def get_motor_temperatures(self):

        """Returns the temperature[C] for each joint"""

        return self.get_list_of_six_values(17)

    def get_joint_modes(self):

        return self.get_list_of_six_values(21)

    def get_actual_joint_voltages(self):

        """Returns the actual voltages[V] of each joint."""

        return self.get_list_of_six_values(33)

    def get_digital_inputs(self):

        """Returns digital inputs as a list of eight bits, where 0 = off and 1 = on"""

        return self.get_decoded_bytes(16)

    def get_digital_outputs(self):

        """Returns digital outputs as a list of eight bits, where 0 = off and 1 = on"""

        return self.get_decoded_bytes(34)

    def get_actual_tool_accelerations(self):

        """Returns actual tool accelerations[m/s^2] in x,y,z."""

        if self.cyclic is not True:
            self.read_packages_acyclic()

        acc_x , acc_y , acc_z = None, None, None

        # get the first eight bytes for the position
        if self.data[24]["value"] is not None:
            # get cartesian accelerations
            acc_x = struct.unpack("!d", self.data[24]["value"][:8])[0]
            acc_y = struct.unpack("!d", self.data[24]["value"][8:16])[0]
            acc_z = struct.unpack("!d", self.data[24]["value"][16:24])[0]

        return [acc_x,acc_y,acc_z]

    def get_main_voltage(self):

        "Returns voltage[V] of the main board."

        if self.cyclic is not True: self.read_packages_acyclic()
        return struct.unpack("!d" , self.data[30]["value"]) if self.data[30]["value"] is not None else None

    def get_robot_voltage(self):

        """Return the voltage[V] of the robot."""

        if self.cyclic is not True: self.read_packages_acyclic()
        return struct.unpack("!d" , self.data[31]["value"]) if self.data[31]["value"] is not None else None

    def get_robot_current(self):

        """Returns the current[A] of the robot."""

        if self.cyclic is not True: self.read_packages_acyclic()
        return struct.unpack("!d" , self.data[32]["value"]) if self.data[32]["value"] is not None else None

    def get_program_state(self):

        """Returns the state of the program: 0 - off , 1 - on"""

        if self.cyclic is not True: self.read_packages_acyclic()
        return struct.unpack("!d" , self.data[35]["value"]) if self.data[35] is not None else None

# --------------------------------- STATUS FUNCTIONS -----------------------------------

    def is_moving(self):

        """Returns True if the UR is moving."""
        

        sum = 0
        #f = self.get_actual_joint_velocities()
        #print(f)
        for v in self.get_actual_joint_velocities():
            if v < 0.05:
                v = 0.0
            sum +=v**2
        

        #print(sum)
        data_package = 0
        return True if sum > 0 else False

    def position_reached(self, target, mode):
        toleranz = 10.0e-5
        #toleranz = 0
        pos_reached = False
        
        if mode == 1: # Method operate with joint coordinates
            actual_values = self.get_actual_joint_coordinates() 
        elif mode == 2: # Method operates with tool coordinates
            actual_values = self.get_actual_tool_coordinates()

        
        target_values = target #self.get_target_joint_coordinates()

        #actual_values= self.get_actual_joint_velocities()
        #target_values= self.get_target_joint_velocities()
        
        #print("Ist-Koordinaten: " + str(actual_values))
        #print("Soll-Koordinaten: " + str(target_values))
        
        for n in range(len(actual_values)):

            tmp = round((abs(actual_values[n]- target_values[n])), 4)

            
            if tmp > toleranz:
                #print("not reached")
                pos_reached = False
                break
            
            else:
                #print("reached")
                pos_reached = True

        """Returns True if the UR reached its desired position."""

        # only if target and actual pose are the same
        return pos_reached
    
    def position_reached_acc(self):
        toleranz = 0
        #toleranz = 0
        pos_reached = False
        
        actual_values= self.get_actual_joint_velocities()
        target_values= self.get_target_joint_velocities()
        
        print("Ist-Koordinaten: " + str(actual_values))
        print("Soll-Koordinaten: " + str(target_values))

        for n in range(len(actual_values)):

            tmp = abs(actual_values[n]- target_values[n])
            
            if tmp > toleranz:
                #print("not reached")
                pos_reached = False
                break
            
            else:
                #print("reached")
                pos_reached = True

        """Returns True if the UR reached its desired position."""

        # only if target and actual pose are the same
        return pos_reached

# --------------------------------- SETTER FUNCTIONS -----------------------------------

    def set_digital_outputs(self , list_of_bits):

        """Sets the digital outputs according to the (list_of_bits)."""

        # cut out list of bits
        list_of_bits = list_of_bits[:8] if len(list_of_bits) > 8 else list_of_bits

        for i in range(len(list_of_bits)):
            if list_of_bits[i] == 0: self.execute_script_command("set_standard_digital_out("+str(i)+",False)")
            elif list_of_bits[i] == 1: self.execute_script_command("set_standard_digital_out("+str(i)+",True)")

    def set_tool_digital_outputs(self , list_of_bits):

        """Sets the digital outputs of the tool according to (list_of_bits)."""

        # cut out list of bits
        list_of_bits = list_of_bits[:2] if len(list_of_bits) > 2 else list_of_bits

        for i in range(len(list_of_bits)):
            if list_of_bits[i] == 0: self.execute_script_command("set_tool_digital_out("+str(i)+",False)")
            elif list_of_bits[i] == 1: self.execute_script_command("set_tool_digital_out("+str(i)+",True)")

    def set_TCP(self, pose):

        """Sets the tool-center-point to the given (pose)[m & rad]."""

        self.execute_script_command("set_tcp("+str(pose)+")")

    def set_payload(self, mass):

        """Configures the mass[kg] of the payload."""

        self.execute_script_command("set_payload("+str(mass)+")")

    def set_message(self , message):

        """Adds a message to the log tab of the UR."""

        self.execute_script_command("textmsg("+message+")")

# -------------------------------- MOVE FUNCTIONS ---------------------------------------

    def move_linear(self , pose , a = 1.2 , v = 0.025 , t = 0 , r = 0):

        """Move linear to (pose) with constant velocity (v)[m/s] and acceleration (a)[m/s^2] in tool-space."""

        self.execute_script_command("movel(p"+str(pose)+","+str(a)+","+str(v)+")")

    def move_circular(self, pose_via , pose_to, a=1.2, v=0.025 , r = 0):

        """Move circular to (pose_to) through (pose_via)."""

        self.execute_script_command("movec(p"+str(pose_via)+",p"+str(pose_to)+","+str(a)+","+str(v)+","+str(r)+")")

    def move_process(self, pose , a=1.2 , v = 0.025 , r=0):

        """Move linear and blend circular to (pose)[m & rad] given in tool-space."""

        self.execute_script_command("movep(p"+str(pose)+","+str(a)+","+str(v)+","+str(r)+")")

    def move_joints(self, joint_pose , a=1.2 , v = 0.025 , r=0):

        """Move to (joint_pose)[m & rad] given in joint-space[rad]."""

        self.execute_script_command("movej("+str(joint_pose)+","+str(a)+","+str(v)+","+str(r)+")")

    def move__linear_with_speed(self, speed_vector, a = 0.1 , t = 2):

        """Accelerates with (a)[m/s^2] and moves with constant velocity[m/s] in tool-space for (t) seconds."""
        self.execute_script_command("speedl("+str(speed_vector)+","+str(a)+","+str(t)+")")

    def move_joints_with_speed(self, speed_vector, a = 0.1 , t = 2):

        """Accelerates with (a)[m/s^2] and move with constant velocity[m/s] in joint-space for (t) seconds."""
        self.execute_script_command("speedj("+str(speed_vector)+","+str(a)+","+str(t)+")")

    def move_relative(self , relative_pose , a = 1.2 , v = 0.025 , t = 0 , r = 0):

        """Moves linear relative to the current pose. [0,0,0.03,0,0,0] would move the UR 3cm up in z-direction."""
        current_position = self.get_actual_tool_coordinates()

        for i in range(len(current_position)):
            current_position[i] += relative_pose[i]

        self.move_linear(current_position,a,v,t,r)

    def move_joints_relative(self , relative_pose , a = 1.2 , v = 0.025 , t = 0 , r = 0):

        """Move joints relative to current joint pose[rad] according to (relative_pose)[rad]."""
        current_joints = self.get_actual_joint_coordinates()

        for i in range(len(relative_pose)):
            current_joints[i] += relative_pose[i]

        self.move_joints(current_joints , a , v, t , r)

    def servo_joints(self, joint_pose , lookahead_time = 0.1 , gain = 300):

        """Servo to positions given in joint-space[rad].

        Args:
            joint_pose(array of doubles): represents the six angles of each joint in radians.
            lookahead_time(double): smoothens out the trajectory with range [0.03s , 0.2s]
            gain(double): proportional gain for following target with range [100 , 2000]
        """

        self.execute_script_command("servoj(get_inverse_kin(p"+str(joint_pose)+"),lookahead_time=0.2,gain=100)")

    def stop(self , a = 1.2):

        """Stops all movement with an optional acceleration[m/s^2]."""

        self.execute_script_command("stopj("+str(a)+")")
        self.execute_script_command("stopl("+str(a)+")")

# -------------------------------- HELPER FUNCTIONS -------------------------------------

    def get_decoded_bytes(self , index):

        """Converts encoded bytes into a eight bit long list representing the states of the digital inputs and outputs."""

        if self.cyclic is not True:
            self.read_packages_acyclic()

        result = None

        # get binary values
        if self.data[index]["value"] is not None:

            # get the binary representation
            binary = str(bin(int.from_bytes(self.data[index]["value"] , byteorder="big")))[2:]

            # check the first 8 bits
            if int(binary[7::-1] , 2) == 0:  result = [0 , 0 , 0 , 0 , 0 , 0 , 0 , 0]
            elif int(binary[7::-1] , 2) > 1: result = [1 , 0 , 0 , 0 , 0 , 0 , 0 , 0]
            else:

                # create empty list
                result = [0] * 8

                # get the highest number
                b_index = int(binary[8:11] , 2) + 1
                result[b_index] = 1

                for i in range(len(binary) - 11):
                    if int(binary[11 + i:  12 + i]) == 1:
                        result[b_index - (i + 1)] = 1

        return result

    def execute_script_command(self, cmd):

        """Sends the UR-script command (cmd) to be executed."""

        cmd_socket = None

        try:
            # open up a new socket for the communication
            cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # connect to UR with port 30003
            cmd_socket.connect((self.HOST_IP, 30003))

            cmd += "\n"

            # send command
            cmd_socket.send(cmd.encode())

        except socket.error as socketError:
            print(socketError)
        finally:

            # always close the connection after sending command
            cmd_socket.close()

    def get_list_of_six_values(self , index):

        """Divides a 48 byte long data package into six float values."""
        
        if self.cyclic is not True:
            self.read_packages_acyclic()

        j1, j2, j3 = None, None, None
        j4, j5, j6 = None, None, None
        #print(self.data[index])
        if len(self.data[index]["value"]) == 48: 
            # get the first eight bytes for the position
            if self.data[index]["value"] is not None:
                # get cartesian positions
                j1 = struct.unpack("!d", self.data[index]["value"][:8])[0]
                j2 = struct.unpack("!d", self.data[index]["value"][8:16])[0]
                j3 = struct.unpack("!d", self.data[index]["value"][16:24])[0]

                # get axis angles
                j4 = struct.unpack("!d", self.data[index]["value"][24:32])[0]
                j5 = struct.unpack("!d", self.data[index]["value"][32:40])[0]
                j6 = struct.unpack("!d", self.data[index]["value"][40:48])[0]
        else: 
            print("Couldn't read fast enough.")
        return [j1,j2,j3,j4,j5,j6]

    def get_pose_in_radians(self , pose_in_degrees):

        """Converts a pose given in degrees into a pose given in radians."""
        for i in range(3,6):
            pose_in_degrees[i] = math.radians(pose_in_degrees[i])

        return pose_in_degrees

    def get_joint_pose_in_radians(self, joint_pose_in_degrees):

        """Converts a joint pose in degrees into a joint pose in radians."""
        for i in range(6):
            joint_pose_in_degrees[i] = math.radians(joint_pose_in_degrees[i])

        return joint_pose_in_degrees
