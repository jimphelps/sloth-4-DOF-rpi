#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import PWM
import time
from threading import Thread
import socket
import RPi.GPIO as GPIO

# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the PWM device using the default address
pwm = PWM(0x40)


def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  print "%d us per period" % pulseLength
  pulseLength /= 4096                     # 12 bits of resolution
  print "%d us per bit" % pulseLength
  pulse *= 1000
  pulse /= pulseLength
  pwm.setPWM(channel, 0, pulse)


class myrobot():
  def __init__(self):
    self.LL = 330
    self.UL = 380

    self.LL_default = 350 #
    self.LR_default = 430 #on augmente, l'interieur se baisse
    self.UL_default = 380
    self.UR_default = 400


    self.LR = 430
    self.UR = 380

    self.vitesse = 7
    self.fonctionne = True

    self.distance_obstacle=100

    self.LL_goal= self.LL_default
    self.LR_goal= self.LR_default
    self.UL_goal = self.UL_default
    self.UR_goal  = self.UR_default

    self.active_action_dict = {}
    self.command = "forward"
    self.step = 0

    self.forward = {0:[0,-40,0,-20],1:[30,-40,30,-20],2:[30,0,30,0],3:[0,20,0,40],4:[-30,20,-30,40],5:[-30,0,-30,0], 6:[0,0,0,0]}
    self.turn = { 0:[-40,0,-20,0],1 :[-40,30,-20,30], 2:[0,30,0,30],3:[30,0,30,0],4:[0,0,0,0]}
    self.dance1 = { 0:[0,-20,0,0],  1:[0,-40,0,20], 2:[0,-20,0,40], 3:[0,0,0,20],  4:[0,0,0,0], 5:[0,0,0,20],   6:[0,-20,0,40], 7:[0,-40,0,20], 8:[0,-20,0,0], 9:[0,0,0,0],  }

    self.dance2 = {
    # //slide to the left 0-4
    1:[0,-20,0,0],  
    2:[0,-40,0,20],
    3:[0,-20,0,40],
    4:[0,0,0,20],
    5:[0,0,0,0],
        
    # //slide to the right 5-9
    6:[0,0,0,20],  
    7:[0,-20,0,40],
    8:[0,-40,0,20],
    9:[0,-20,0,0],
    10:[0,0,0,0]}
    # };
    self.dance3 = {  0:[20,0,40,0],
    1:[20,-30,40,-30], 
    2:[20,-30,10,-30],
    3:[20,-30,40,-30], 
    4:[20,-30,10,-30],
    5:[20,-30,40,-30], 
    6:[20,0,40,-30],
    7:[20,80,40,-30],
    8:[20,0,40,-30],
    9:[20,-80,40,-30],        
    10:[20,0,40,-30],
    11:[20,80,40,-30],
    12:[20,0,40,-30],
    13:[20,-30,40,-30],    
    14:[20,0,40,0],
    15:[0,0,0,0], 
       
    # //right foot support 16-31
    16:[-40,0,-20,0],
    17:[-40,40,-20,30], 
    18:[-20,40,-20,30], 
    19:[-40,40,-20,30], 
    20:[-20,40,-20,30], 

    21:[-40,40,-20,30], 
    22:[-40,40,-20,0],
    23:[-40,40,-20,-80],
    24:[-40,40,-20,0],
    25:[-40,40,-20,80],        
    26:[-40,40,-20,0],
    27:[-40,40,-20,-80],
    28:[-40,40,-20,0],
    29:[-40,40,-20,30],    
    30:[-40,0,-20,0],
    31:[0,0,0,0],
    }
    # # const int array_dance2[num_dance2][4] =
    # # { 
    # # //left foot support 0-15
    # 0:[20,0,40,0],
    # 0:[20,-30,40,-30], 
    # 0:[20,-30,10,-30],
    # 0:[20,-30,40,-30], 
    # 0:[20,-30,10,-30],

    # 0:[20,-30,40,-30], 
    # 0:[20,0,40,-30],
    # 0:[20,80,40,-30],
    # 0:[20,0,40,-30],
    # 0:[20,-80,40,-30],        
    # 0:[20,0,40,-30],
    # 0:[20,80,40,-30],
    # 0:[20,0,40,-30],
    # 0:[20,-30,40,-30],    
    # 0:[20,0,40,0],
    # 0:[0,0,0,0], 
         
    # # //right foot support 16-31
    # 0:[-40,0,-20,0],
    # 0:[-40,40,-20,30], 
    # 0:[-20,40,-20,30], 
    # 0:[-40,40,-20,30], 
    # 0:[-20,40,-20,30], 

    # 0:[-40,40,-20,30], 
    # 0:[-40,40,-20,0],
    # 0:[-40,40,-20,-80],
    # 0:[-40,40,-20,0],
    # 0:[-40,40,-20,80],        
    # 0:[-40,40,-20,0],
    # 0:[-40,40,-20,-80],
    # 0:[-40,40,-20,0],
    # 0:[-40,40,-20,30],    
    # 0:[-40,0,-20,0],
    # 0:[0,0,0,0],
    # # };


    # # const int array_dance3[num_dance3][4] =
    # # { 
    # 0:[0,-40,0,0],
    # 0:[20,-30,20,20],
    # 0:[40,0,40,30],   
    # 0:[0,0,0,40],
    # 0:[-20,-20,-20,30],
    # 0:[-40,-30,-40,0],

    # 0:[0,-40,0,0],
    # 0:[0,0,0,0],
    # # };

    # # const int array_dance4[num_dance4][4] =
    # # {     
    # 0:[0,-20,0,20],   
    # 0:[0,0,0,0],
    # 0:[0,-20,0,20],
    # 0:[0,0,0,0],
    # 0:[0,-20,0,20],
    # 0:[0,0,0,0],
    # 0:[0,-20,0,20],
    # 0:[0,0,0,0],
        
    # 0:[0,-50,0,50],
    # 0:[0,0,0,0],    
    # 0:[0,-50,0,50],
    # 0:[0,0,0,0],    
    # 0:[0,-50,0,50],
    # 0:[0,0,0,0],    
    # 0:[0,-50,0,50],
    # 0:[0,0,0,0],
        
    # 0:[0,-40,0,40],
    # 0:[0,-50,0,50],
    # 0:[0,-60,0,60],
    # 0:[0,0,0,0,],
    # # };

    # # const int num_dance5 = 17;
    # # const int array_dance5[num_dance5][4] =
    # # { 
    # 0:[35,0,15,0],    
    # 0:[35,30,15,30],   
    # 0:[-35,30,15,30],   
    # 0:[-20,0,15,0],
    # 0:[0,0,0,0],

    # 0:[0,-40,0,40],    
    # 0:[-30,-40,-20,40],
    # 0:[0,-40,0,40],
    # 0:[20,-40,30,40],    

        
    # 0:[0,-40,0,40],    
    # 0:[20,-40,-20,40],   
    # 0:[20,-20,-20,20],
    # 0:[20,0,-20,0],   
    # 0:[-20,-10,20,10],
    # 0:[-10,-30,10,30],

    # 0:[0,-40,0,40],
    # 0:[0,0,0,0],
    # # };

    # # const int num_dance6 = 32;      
    # # const int array_dance6[num_dance6][4] =
    # # {   
    # 0:[0,-40,0,-20],       
    # 0:[25,-40,18,-20],
    # 0:[25,0,18,0],
    # 0:[0,20,0,40],
    # 0:[-18,20,-25,40],
    # 0:[-18,0,-25,0],
          
    # 0:[0,-40,0,-20],      
    # 0:[25,-40,18,-20],
    # 0:[0,0,0,0],
    # 0:[0,20,0,40],
    # 0:[-18,20,-25,40],
    # 0:[0,0,0,0],

    # 0:[0,-40,0,-20],      
    # 0:[-25,-40,-18,-20],
    # 0:[-25,0,-18,0],
    # 0:[0,20,0,40],
    # 0:[18,20,25,40],
    # 0:[18,0,25,0],

    # 0:[30,-40,30,-20],
    # 0:[30,0,30,0],
    # 0:[0,20,0,40],
    # 0:[-30,20,-30,40],
    # 0:[-30,0,-30,0],
      
    # 0:[0,-40,0,-20],        
    # 0:[-30,-40,-30,-20],
    # 0:[-30,0,-30,0],
    # 0:[0,20,0,40],
    # 0:[30,20,30,40],
    # 0:[30,0,30,0],

    # 0:[15,0,15,0],
    # 0:[0,0,0,0},
    # # };



  def put_start_pos(self):
    self.UL = self.UL_default
    self.UR = self.UR_default
    self.LL = self.LL_default
    self.LR = self.LR_default
    self.do_it()

  def setDistance(self,distance):
    self.distance_obstacle=float(distance)

  def getDistance(self):
    return self.distance_obstacle

  def reset_avancer(self):
    self.nb_forward =0

  def reset_turn(self):
    self.nb_turn=0

  def do_action(self):
    objectifs =self.active_action_dict[self.step]
    self.UL_goal= self.UR_default +objectifs[0]
    self.LR_goal= self.LR_default +objectifs[1]
    self.UR_goal= self.UL_default +objectifs[2]
    self.LL_goal= self.LL_default +objectifs[3]   
    if self.step ==len(self.active_action_dict)-1:
      self.step=0

    if self.get_goalok():
      self.step = self.step +1
    self.goto_goal()


  def setCommand(self,command):
    self.step = 0
    self.command=command

    if self.command == "forward":
      self.active_action_dict = self.forward
    elif self.command == "turn":
      self.active_action_dict = self.turn
    elif self.command == "dance1":
      self.active_action_dict = self.dance1
    else:
      self.active_action_dict = self.forward


  def getCommand(self):
    return self.command

  def goto_goal(self):
    if abs(self.LL_goal - self.LL) <= self.vitesse+1:
      self.LL = self.LL_goal
    else:
      if self.LL_goal > self.LL:
        self.LL += self.vitesse
      else:
        self.LL -= self.vitesse

    if abs(self.LR_goal-self.LR) <= self.vitesse+1:
      self.LR=self.LR_goal
    else:
      if self.LR_goal > self.LR:
        self.LR += self.vitesse
      else:
        self.LR -= self.vitesse

    if abs(self.UR_goal-self.UL) <= self.vitesse+1:
      self.UL=self.UR_goal
    else:
      if self.UR_goal > self.UL:
        self.UL += self.vitesse
      else:
        self.UL -= self.vitesse

    if abs(self.UL_goal - self.UR) <= self.vitesse+1:
      self.UR=self.UL_goal
    else:
      if self.UL_goal > self.UR:
        self.UR += self.vitesse
      else:
        self.UR -= self.vitesse

  def get_goalok(self):
    #verifier que les objectifs sont atteints
    if self.LL==self.LL_goal and self.LR==self.LR_goal and self.UL==self.UR_goal and self.UR==self.UL_goal:
      return True
    else:
      return False


  def roboton(self):
    return self.fonctionne

  def stoprobot(self):
    self.fonctionne = False

  def do_it(self):
    pwm.setPWM(1, 0, self.UL)
    pwm.setPWM(0, 0, self.LL)
    pwm.setPWM(2, 0, self.LR)# d_piedincline)
    pwm.setPWM(3, 0, self.UR )#d_hanche_incl)




class ThRobot(Thread):
  def __init__(self):
    Thread.__init__(self)
  def run(self): 
    global robot
    robot.put_start_pos()
    robot.setCommand("forward")
    time.sleep(5)
    while robot.roboton():
      robot.do_action()

      if robot.getDistance()<15 and robot.getCommand()=="forward":
        robot.setCommand("turn")
      if robot.getDistance()>20 and robot.getCommand()=="turn":
        robot.setCommand("forward")


      robot.do_it()
      time.sleep(0.02)


class ThSocket(Thread):
  def __init__(self):
    Thread.__init__(self)
  def run(self): 
    global robot
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

    while robot.roboton():
      data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
      print("distance:", data)
      robot.setDistance(data)
    sock.close()

class ThUltrasonic(Thread):
  def __init__(self):
    Thread.__init__(self)
  def run(self): 
    global robot
    GPIO.setmode(GPIO.BCM)
    Trig = 23          # Entree Trig du HC-SR04 branchee au GPIO 23
    Echo = 24         # Sortie Echo du HC-SR04 branchee au GPIO 24

    GPIO.setup(Trig,GPIO.OUT)
    GPIO.setup(Echo,GPIO.IN)

    GPIO.output(Trig, False)

    while robot.roboton():
       time.sleep(1)       # On la prend toute les 1 seconde

       GPIO.output(Trig, True)
       time.sleep(0.00001)
       GPIO.output(Trig, False)

       while GPIO.input(Echo)==0:  ## Emission de l'ultrason
         debutImpulsion = time.time()

       while GPIO.input(Echo)==1:   ## Retour de l'Echo
         finImpulsion = time.time()

       distance = round((finImpulsion - debutImpulsion) * 340 * 100 / 2, 1)  ## Vitesse du son = 340 m/s

       print "La distance est de : ",distance," cm"
       robot.setDistance(distance)

    GPIO.cleanup()


class ThSocketCmd(Thread):
  def __init__(self):
    Thread.__init__(self)
  def run(self): 
    global robot
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5006
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

    while robot.roboton():
      data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
      print("Command:", data)
      robot.setCommand(data)
    sock.close()

global robot 

robot = myrobot()
thread_1 = ThRobot()
thread_2 = ThUltrasonic()
# # Lancement des threads
thread_1.start()
thread_2.start()
# # Attend que les threads se terminent
# thread_1.join()



pwm.setPWMFreq(60)                        # Set frequency to 60 Hz
i=0
while (i<10):
  # Change speed of continuous servo on channel O
  # print("Pos 0: g_piedplat{} g_hanche_droit:{}  d_piedplat{} d_hanche_droit:{}  ".format(g_piedplat,g_hanche_droit, d_piedplat,d_hanche_droit))
  # pwm.setPWM(0, 0, g_piedplat)
  # pwm.setPWM(1, 0, g_hanche_droit)
  # # pwm.setPWM(2, 0, d_piedplat)
  # # pwm.setPWM(3, 0, d_hanche_droit)

  # time.sleep(10)
  # print("Pos 0: g_piedincline{} g_hanche_incl:{} d_piedincline{} d_hanche_incl:{}".format(g_piedincline, g_hanche_incl, d_piedincline, d_hanche_incl))
  # pwm.setPWM(0, 0, g_piedincline)
  # pwm.setPWM(1, 0, g_hanche_incl)
  # pwm.setPWM(2, 0, d_piedincline)
  # pwm.setPWM(3, 0, d_hanche_incl)
  print('robot running...{}'.format(i))
  time.sleep(10)
  i=i+1
print("bye")
robot.put_start_pos()
robot.stoprobot()
# pwm.setPWM(0, 4096, 0) 
thread_1.join()
thread_2.join()
