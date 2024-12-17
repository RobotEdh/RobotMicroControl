import numpy as np
import matplotlib.pyplot as plt

TlogPID = np.zeros((10000,3),dtype=int)
TlogPIDRoll = np.zeros((10000,12),dtype=int)
TlogPIDPitch = np.zeros((10000,12),dtype=int)
TlogPIDYaw = np.zeros((10000,12),dtype=int)
TlogMotor = np.zeros((10000,6),dtype=int)

fichier = open("F:\log.txt","rb")

char0 = 'a'
char1 = 'b'
iPID = 0
iPIDRoll = 0
iPIDPitch = 0
iPIDYaw = 0
iMotor = 0
NbTick = 0
MaxDT = 0
piddumpsize = 0
motordumpsize = 0
cc = 0
eof = False
while (not eof):
   foundPID = False
   foundMotor = False
   while (not foundPID and not foundMotor and not eof):
       char0 = fichier.read(1)
       char1 = fichier.read(1)
       if ((len(char0) == 0) or (len(char1) == 0)):
           eof = True
       elif ((ord(char0) == 0xFA) and (ord(char1) == 0xFB)):
           cc=cc+1
           if (cc > 0):
               foundPID = True
               print("PID begin dump")
               piddumpsize = 2
           else:
               print("PID ignore dump")
       elif ((ord(char0) == 0xFB) and (ord(char1) == 0xFC)):
           foundMotor = True
           print("Motor begin dump")
           motordumpsize = 2
       elif ((ord(char0) > 0x1F) and (ord(char0) < 0x7E) and (ord(char1) > 0x1F) and (ord(char1) < 0x7E)):
           fichier.seek(-2,1)
           line = fichier.readline()
           print(line.strip())
    
   found = False
   while (not found and not eof ):
       char0 = fichier.read(1)
       char1 = fichier.read(1)
       if ((len(char0) == 0) or (len(char1) == 0)):
           eof = True
       elif ((ord(char0) == 0xFC) and (ord(char1) == 0xFD)): # PID
           found = True
           print("PID end dump")
           piddumpsize = piddumpsize+2+4 # 6 bytes stop
           print("PID dumpsize:",piddumpsize)
           char0 = fichier.read(4)  # 4 fillers
       elif ((ord(char0) == 0xFD) and (ord(char1) == 0xFE)): # Motors
           found = True
           print("Motor end dump")
           motordumpsize = motordumpsize+2+4 # 6 bytes stop
           print("motor dump size:",motordumpsize)
           char0 = fichier.read(4)  # 4 fillers
       elif (foundMotor):
              MSB = int.from_bytes(char1, byteorder='little') << 8
              LSB = int.from_bytes(char0, byteorder='little')
              twobytes0 = (MSB | LSB).to_bytes(2, byteorder='little',signed=False) # throttle
              twobytes1 = fichier.read(2) # motor0
              twobytes2 = fichier.read(2) # motor1
              twobytes3 = fichier.read(2) # motor2
              twobytes4 = fichier.read(2) # motor3
              twobytes5 = fichier.read(2) # tick 
              motordumpsize = motordumpsize+12  # 12 bytes    

              TlogMotor[iMotor,0] = int.from_bytes(twobytes5, byteorder='little',signed=False) # tick
              TlogMotor[iMotor,1] = int.from_bytes(twobytes0, byteorder='little',signed=False) # throttle
              TlogMotor[iMotor,2] = int.from_bytes(twobytes1, byteorder='little',signed=False) # motor0
              TlogMotor[iMotor,3] = int.from_bytes(twobytes2, byteorder='little',signed=False) # motor1
              TlogMotor[iMotor,4] = int.from_bytes(twobytes3, byteorder='little',signed=False) # motor2
              TlogMotor[iMotor,5] = int.from_bytes(twobytes4, byteorder='little',signed=False) # motor3                    
              iMotor = iMotor + 1
       elif ((ord(char0) != 0x00) and (ord(char0) != 0x01) and (ord(char0) != 0x02)):
              found = True
              print("Data error, angle type should be between 0 and 2: ",ord(char0))
       else:
           if (foundPID):
                #print("index: ",int.from_bytes(char0, byteorder='little',signed=False))
                #print("dt: ",int.from_bytes(char1, byteorder='little',signed=False))
                twobytes1 = fichier.read(2) # countESC
                #print("countESC 2 bytes: ",int.from_bytes(twobytes1, byteorder='little',signed=False))               
                twobytes2 = fichier.read(2) # instruction
                #print("instruction 2 bytes: ",int.from_bytes(twobytes2, byteorder='little',signed=True))
                twobytes3 = fichier.read(2) # angle
                twobytes4 = fichier.read(2) # error
                #print("error 2 bytes: ",int.from_bytes(twobytes4, byteorder='little',signed=True))
                twobytes5 = fichier.read(2) # lastError
                #print("lastError 2 bytes: ",int.from_bytes(twobytes5, byteorder='little',signed=True))
                twobytes6 = fichier.read(2) # prop
                #print("prop 2 bytes: ",int.from_bytes(twobytes6, byteorder='little',signed=True))
                twobytes7 = fichier.read(2) # integ
                #print("integ 2 bytes: ",int.from_bytes(twobytes7, byteorder='little',signed=True))
                twobytes8 = fichier.read(2) # derivative
                #print("derivative 2 bytes: ",int.from_bytes(twobytes8, byteorder='little',signed=True))
                twobytes9 = fichier.read(2) # lastDerivative
                #print("lastDerivative 2 bytes: ",int.from_bytes(twobytes9, byteorder='little',signed=True))  
                twobytes10 = fichier.read(2) # anglePID
                #print("anglePID 2 bytes: ",int.from_bytes(twobytes10, byteorder='little',signed=True))                                                                                                  
                twobytes11 = fichier.read(2) # tick
                #print("tick 2 bytes: ",int.from_bytes(twobytes11, byteorder='little',signed=False))                                                                                                  
                piddumpsize = piddumpsize+24 # 24 bytes
                #print("piddumpsize: ",piddumpsize)
                if ((ord(char0) == 0x01) and (int.from_bytes(twobytes10, byteorder='little',signed=False) <-100)): 
                    print("****************pitch: ",int.from_bytes(twobytes3, byteorder='little',signed=True)/10)
                    print("****************pitchPID: ",int.from_bytes(twobytes0, byteorder='little',signed=True)/10)
                    print("****************tick: ",int.from_bytes(twobytes11, byteorder='little',signed=False))
                if ((ord(char0) == 0x00) and (int.from_bytes(twobytes10, byteorder='little',signed=False) < -100)): 
                    print("****************roll: ",int.from_bytes(twobytes3, byteorder='little',signed=True)/10)
                    print("****************rollPID: ",int.from_bytes(twobytes10, byteorder='little',signed=True)/10)
                    print("****************tick: ",int.from_bytes(twobytes11, byteorder='little',signed=False))
  
                if (ord(char0) == 0x00):
                    TlogPID[iPID,0] = int.from_bytes(twobytes11, byteorder='little',signed=False) # tick
                    TlogPID[iPID,1] = int.from_bytes(char1, byteorder='little',signed=True) # dt
                    TlogPID[iPID,2] = int.from_bytes(twobytes1, byteorder='little',signed=False) # countESC

                    NbTick = TlogPID[iPID,0]
                    CountESC = TlogPID[iPID,2]
                    if (TlogPID[iPID,1]>MaxDT):
                        MaxDT = TlogPID[iPID,1]
                    
                    iPID = iPID +1
                 
                if (ord(char0) == 0):                  #Roll
                    TlogPIDRoll[iPIDRoll,0] = int.from_bytes(twobytes11, byteorder='little',signed=False) # tick
                    TlogPIDRoll[iPIDRoll,1] = int.from_bytes(char0, byteorder='little',signed=False) # index
                    TlogPIDRoll[iPIDRoll,2] = int.from_bytes(char1, byteorder='little',signed=True) # dt
                    TlogPIDRoll[iPIDRoll,3] = int.from_bytes(twobytes2, byteorder='little',signed=True) # instruction
                    TlogPIDRoll[iPIDRoll,4] = int.from_bytes(twobytes3, byteorder='little',signed=True) # angle
                    TlogPIDRoll[iPIDRoll,5] = int.from_bytes(twobytes4, byteorder='little',signed=True) # error      
                    TlogPIDRoll[iPIDRoll,6] = int.from_bytes(twobytes5, byteorder='little',signed=True) # lastError
                    TlogPIDRoll[iPIDRoll,7] = int.from_bytes(twobytes6, byteorder='little',signed=True) # prop
                    TlogPIDRoll[iPIDRoll,8] = int.from_bytes(twobytes7, byteorder='little',signed=True) # integ
                    TlogPIDRoll[iPIDRoll,9] = int.from_bytes(twobytes8, byteorder='little',signed=True) # derivative
                    TlogPIDRoll[iPIDRoll,10] = int.from_bytes(twobytes9, byteorder='little',signed=True) # lastDerivative
                    TlogPIDRoll[iPIDRoll,11] = int.from_bytes(twobytes10, byteorder='little',signed=True) # anglePID
                    iPIDRoll = iPIDRoll +1
                elif (ord(char0) == 1):                  #Pitch
                    TlogPIDPitch[iPIDPitch,0] = int.from_bytes(twobytes11, byteorder='little',signed=False) # tick
                    TlogPIDPitch[iPIDPitch,1] = int.from_bytes(char0, byteorder='little',signed=False) # index
                    TlogPIDPitch[iPIDPitch,2] = int.from_bytes(char1, byteorder='little',signed=True) # dt
                    TlogPIDPitch[iPIDPitch,3] = int.from_bytes(twobytes2, byteorder='little',signed=True) # instruction
                    TlogPIDPitch[iPIDPitch,4] = int.from_bytes(twobytes3, byteorder='little',signed=True) # angle
                    TlogPIDPitch[iPIDPitch,5] = int.from_bytes(twobytes4, byteorder='little',signed=True) # error      
                    TlogPIDPitch[iPIDPitch,6] = int.from_bytes(twobytes5, byteorder='little',signed=True) # lastError
                    TlogPIDPitch[iPIDPitch,7] = int.from_bytes(twobytes6, byteorder='little',signed=True) # prop
                    TlogPIDPitch[iPIDPitch,8] = int.from_bytes(twobytes7, byteorder='little',signed=True) # integ
                    TlogPIDPitch[iPIDPitch,9] = int.from_bytes(twobytes8, byteorder='little',signed=True) # derivative
                    TlogPIDPitch[iPIDPitch,10] = int.from_bytes(twobytes9, byteorder='little',signed=True) # lastDerivative
                    TlogPIDPitch[iPIDPitch,11] = int.from_bytes(twobytes10, byteorder='little',signed=True) # anglePID
                    iPIDPitch = iPIDPitch +1
                elif (ord(char0) == 2):                  #Yaw
                    TlogPIDYaw[iPIDYaw,0] = int.from_bytes(twobytes11, byteorder='little',signed=False) # tick
                    TlogPIDYaw[iPIDYaw,1] = int.from_bytes(char0, byteorder='little',signed=False) # index
                    TlogPIDYaw[iPIDYaw,2] = int.from_bytes(char1, byteorder='little',signed=True) # dt
                    TlogPIDYaw[iPIDYaw,3] = int.from_bytes(twobytes2, byteorder='little',signed=True) # instruction
                    TlogPIDYaw[iPIDYaw,4] = int.from_bytes(twobytes3, byteorder='little',signed=True) # angle
                    TlogPIDYaw[iPIDYaw,5] = int.from_bytes(twobytes4, byteorder='little',signed=True) # error      
                    TlogPIDYaw[iPIDYaw,6] = int.from_bytes(twobytes5, byteorder='little',signed=True) # lastError
                    TlogPIDYaw[iPIDYaw,7] = int.from_bytes(twobytes6, byteorder='little',signed=True) # prop
                    TlogPIDYaw[iPIDYaw,8] = int.from_bytes(twobytes7, byteorder='little',signed=True) # integ
                    TlogPIDYaw[iPIDYaw,9] = int.from_bytes(twobytes8, byteorder='little',signed=True) # derivative
                    TlogPIDYaw[iPIDYaw,10] = int.from_bytes(twobytes9, byteorder='little',signed=True) # lastDerivative
                    TlogPIDYaw[iPIDYaw,11] = int.from_bytes(twobytes10, byteorder='little',signed=True) # anglePID
                    iPIDYaw = iPIDYaw +1                  
                        

              

 

print("************************* End log ****************************************")
print("Nb Tick: ",NbTick)
print("count ESC: ",CountESC)
print("count PID: ",iPID)
print("Max dt: ",MaxDT)
print("count Motor: ",iMotor)
print("Roll: ",TlogPIDRoll[355:370,0])
print("Roll: ",TlogPIDRoll[355:370,1])
print("Roll: ",TlogPIDRoll[355:370,2])
print("Roll: ",TlogPIDRoll[355:370,3]/10)
print("Roll: ",TlogPIDRoll[355:370,4]/10)
print("Roll: ",TlogPIDRoll[355:370,5]/10)
print("Roll: ",TlogPIDRoll[355:370,6]/10)
print("Roll: ",TlogPIDRoll[355:370,7]/10)
print("Roll: ",TlogPIDRoll[355:370,8]/10)
print("Roll: ",TlogPIDRoll[355:370,9]/10)
print("Roll: ",TlogPIDRoll[355:370,10]/10)
print("Roll: ",TlogPIDRoll[355:370,11]/10)

plt.title('dt')
plt.plot(TlogPID[0:iPID,0], TlogPID[0:iPID,1],'k*' ,label='dt',linewidth=1,markersize=1)
plt.legend()
plt.show()

plt.title('Angles')
plt.plot(TlogPIDRoll[0:iPIDRoll,0], TlogPIDRoll[0:iPIDRoll,4]/10,'k-' ,label='Roll',linewidth=1,markersize=1)
plt.plot(TlogPIDPitch[0:iPIDPitch,0], TlogPIDPitch[0:iPIDPitch,4]/10,'g-' ,label='Pitch',linewidth=1,markersize=1)
plt.plot(TlogPIDYaw[0:iPIDYaw,0], TlogPIDYaw[0:iPIDRoll,4]/10,'r-' ,label='Yaw',linewidth=1,markersize=1)
plt.legend()
plt.show()


plt.title('Roll')
plt.plot(TlogPIDRoll[0:iPIDRoll,0], TlogPIDRoll[0:iPIDRoll,4]/10,'g*' ,label='Roll',linewidth=1,markersize=1)
plt.legend()
plt.show()

plt.title('Pitch')
plt.plot(TlogPIDPitch[0:iPIDPitch,0], TlogPIDPitch[0:iPIDPitch,4]/10,'g*' ,label='Pitch',linewidth=1,markersize=1)
plt.legend()
plt.show()

plt.title('Yaw')
plt.plot(TlogPIDYaw[0:iPIDYaw,0], TlogPIDYaw[0:iPIDYaw,4]/10,'g*' ,label='Yaw',linewidth=1,markersize=1)
plt.legend()
plt.show()

plt.title('PID Roll')
plt.plot(TlogPIDRoll[0:iPIDRoll,0], TlogPIDRoll[0:iPIDRoll,7]/10,'c-' ,label='P',linewidth=1,markersize=1)
plt.plot(TlogPIDRoll[0:iPIDRoll,0], TlogPIDRoll[0:iPIDRoll,8]/10,'g-' ,label='I',linewidth=1,markersize=1)
plt.plot(TlogPIDRoll[0:iPIDRoll,0], TlogPIDRoll[0:iPIDRoll,9]/10,'y-' ,label='D',linewidth=1,markersize=1)
plt.plot(TlogPIDRoll[0:iPIDRoll,0], TlogPIDRoll[0:iPIDRoll,11]/10,'r-' ,label='PID',linewidth=2,markersize=1)
plt.legend()
plt.show()

plt.title('Pitch')
plt.plot(TlogPIDPitch[0:iPIDPitch,0], TlogPIDPitch[0:iPIDPitch,3]/10,'k-' ,label='PitchInstruction',linewidth=1,markersize=1)
plt.plot(TlogPIDPitch[0:iPIDPitch,0], TlogPIDPitch[0:iPIDPitch,4]/10,'g-' ,label='Pitch',linewidth=1,markersize=1)
plt.plot(TlogPIDPitch[0:iPIDPitch,0], TlogPIDPitch[0:iPIDPitch,11]/10,'r-' ,label='PitchPID',linewidth=2,markersize=1)
plt.legend()
plt.show()

plt.title('PID Pitch')
plt.plot(TlogPIDPitch[0:iPIDPitch,0], TlogPIDPitch[0:iPIDPitch,7]/10,'c-' ,label='P',linewidth=1,markersize=1)
plt.plot(TlogPIDPitch[0:iPIDPitch,0], TlogPIDPitch[0:iPIDPitch,8]/10,'g-' ,label='I',linewidth=1,markersize=1)
plt.plot(TlogPIDPitch[0:iPIDPitch,0], TlogPIDPitch[0:iPIDPitch,9]/10,'y-' ,label='D',linewidth=1,markersize=1)
plt.plot(TlogPIDPitch[0:iPIDPitch,0], TlogPIDPitch[0:iPIDPitch,11]/10,'r-' ,label='PID',linewidth=2,markersize=1)
plt.legend()
plt.show()

plt.title('Yaw')
plt.plot(TlogPIDYaw[0:iPIDYaw,0], TlogPIDYaw[0:iPIDYaw,3]/10,'k-' ,label='YawInstruction',linewidth=1,markersize=1)
plt.plot(TlogPIDYaw[0:iPIDYaw,0], TlogPIDYaw[0:iPIDYaw,4]/10,'g-' ,label='Yaw',linewidth=1,markersize=1)
plt.plot(TlogPIDYaw[0:iPIDYaw,0], TlogPIDYaw[0:iPIDYaw,11]/10,'r-' ,label='YawPID',linewidth=2,markersize=1)
plt.legend()
plt.show()

plt.title('PID Yaw')
plt.plot(TlogPIDYaw[0:iPIDYaw,0], TlogPIDYaw[0:iPIDYaw,7]/10,'c-' ,label='P',linewidth=1,markersize=1)
plt.plot(TlogPIDYaw[0:iPIDYaw,0], TlogPIDYaw[0:iPIDYaw,8]/10,'g-' ,label='I',linewidth=1,markersize=1)
plt.plot(TlogPIDYaw[0:iPIDYaw,0], TlogPIDYaw[0:iPIDYaw,9]/10,'y-' ,label='D',linewidth=1,markersize=1)
plt.plot(TlogPIDYaw[0:iPIDYaw,0], TlogPIDYaw[0:iPIDYaw,11]/10,'r-' ,label='PID',linewidth=2,markersize=1)
plt.legend()
plt.show()


plt.title('Motors')
plt.plot(TlogMotor[0:iMotor,0],TlogMotor[0:iMotor,1],'k-',label='Throttle',linewidth=1,markersize=1)
plt.plot(TlogMotor[0:iMotor,0],TlogMotor[0:iMotor,2],'c-',label='Front Left',linewidth=1,markersize=1)
plt.plot(TlogMotor[0:iMotor,0],TlogMotor[0:iMotor,3],'g-',label='Front Right',linewidth=1,markersize=1)
plt.plot(TlogMotor[0:iMotor,0],TlogMotor[0:iMotor,4],'b-',label='Rear Right',linewidth=1,markersize=1)
plt.plot(TlogMotor[0:iMotor,0],TlogMotor[0:iMotor,5],'y-',label='Rear Left',linewidth=1,markersize=1)
plt.legend()
plt.show()

fichier.close()
