import numpy as np
import matplotlib.pyplot as plt

TlogPID = np.zeros((10000,9),dtype=int)
TlogPIDRoll = np.zeros((10000,8),dtype=int)
TlogPIDPitch = np.zeros((10000,8),dtype=int)
TlogPIDYaw = np.zeros((10000,8),dtype=int)
TlogMotor = np.zeros((10000,6),dtype=int)

fichier = open("F:\log.txt","rb")

char0 = 'a'
char1 = 'b'
iPID = 0
iPIDRoll = 0
iPIDPitch = 0
iPIDYaw = 0
iMotor = 0
MaxTick = 0
iMaxsampleTime = 0

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
           foundPID = True
           print("PID begin dump")
           piddumpsize = 0
       elif ((ord(char0) == 0xFB) and (ord(char1) == 0xFC)):
           foundMotor = True
           print("Motor begin dump")
           motordumpsize = 0
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
           print ("PID dumpsize:",piddumpsize)
           char0 = fichier.read(6)  # 6 fillers
       elif ((ord(char0) == 0xFD) and (ord(char1) == 0xFE)): # Motors
           found = True
           print("Motor end dump")
           print ("motor dump size:",motordumpsize)
           char0 = fichier.read(4)  # 4 fillers
       elif (foundMotor):
              char2 = fichier.read(1) # motor1
              char3 = fichier.read(1) # motor2
              char4 = fichier.read(1) # motor3
              fourbytes = fichier.read(4) # tick 32 bytes
              if (int.from_bytes(fourbytes, byteorder='little',signed=True) > 0):
                motordumpsize = motordumpsize+2+3+4    
                TlogMotor[iMotor,0] = int.from_bytes(fourbytes, byteorder='little',signed=False)
                TlogMotor[iMotor,1] = int.from_bytes(char0, byteorder='little',signed=False)
                if (TlogMotor[iMotor,1] < 120):
                    TlogMotor[iMotor,1] = 120
                TlogMotor[iMotor,2] = int.from_bytes(char1, byteorder='little',signed=False)
                TlogMotor[iMotor,3] = int.from_bytes(char2, byteorder='little',signed=False)
                TlogMotor[iMotor,4] = int.from_bytes(char3, byteorder='little',signed=False)
                TlogMotor[iMotor,5] = int.from_bytes(char4, byteorder='little',signed=False)
                if (TlogMotor[iMotor,0]>MaxTick):
                    MaxTick = TlogMotor[iMotor,0]                    
                iMotor = iMotor + 1;
       elif ((ord(char0) != 0x00) and (ord(char0) != 0x01) and (ord(char0) != 0x02)):
              found = True
              print("Data error, angle type should be between 0 and 2: ",ord(char0))
       else:
           if (foundPID):
              char2 = fichier.read(1) # RC_commandRP
              char3 = fichier.read(1) # error
              char4 = fichier.read(1) # sum_error
              char5 = fichier.read(1) # delta_error
              char6 = fichier.read(1) # anglePID
              char7 = fichier.read(1) # sampleTime
              fourbytes = fichier.read(4) # tick 32 bytes
              if (int.from_bytes(fourbytes, byteorder='little',signed=True) > 0):    
                piddumpsize = piddumpsize+2+6+4 
                TlogPID[iPID,0] = int.from_bytes(fourbytes, byteorder='little',signed=True)
                TlogPID[iPID,1] = int.from_bytes(char0, byteorder='little',signed=True)
                TlogPID[iPID,2] = int.from_bytes(char1, byteorder='little',signed=True)
                TlogPID[iPID,3] = int.from_bytes(char2, byteorder='little',signed=True)
                TlogPID[iPID,4] = int.from_bytes(char3, byteorder='little',signed=True)
                TlogPID[iPID,5] = int.from_bytes(char4, byteorder='little',signed=True)        
                TlogPID[iPID,6] = int.from_bytes(char5, byteorder='little',signed=True)
                TlogPID[iPID,7] = int.from_bytes(char6, byteorder='little',signed=True)
                TlogPID[iPID,8] = int.from_bytes(char7, byteorder='little',signed=True)
                if (TlogPID[iPID,0]>MaxTick):
                    MaxTick = TlogPID[iPID,0]
                if (TlogPID[iPID,8]>20):
                    iMaxsampleTime = iMaxsampleTime + 1
                iPID = iPID +1
                if (ord(char0) == 0):                  #Roll
                    TlogPIDRoll[iPIDRoll,0] = int.from_bytes(fourbytes, byteorder='little',signed=True)
                    TlogPIDRoll[iPIDRoll,1] = int.from_bytes(char1, byteorder='little',signed=True)
                    TlogPIDRoll[iPIDRoll,2] = int.from_bytes(char2, byteorder='little',signed=True)
                    TlogPIDRoll[iPIDRoll,3] = int.from_bytes(char3, byteorder='little',signed=True)
                    TlogPIDRoll[iPIDRoll,4] = int.from_bytes(char4, byteorder='little',signed=True)
                    TlogPIDRoll[iPIDRoll,5] = int.from_bytes(char5, byteorder='little',signed=True)
                    TlogPIDRoll[iPIDRoll,6] = int.from_bytes(char6, byteorder='little',signed=True)
                    TlogPIDRoll[iPIDRoll,7] = int.from_bytes(char7, byteorder='little',signed=True)
                    iPIDRoll = iPIDRoll +1
                elif (ord(char0) == 1):                  #Pitch
                    TlogPIDPitch[iPIDPitch,0] = int.from_bytes(fourbytes, byteorder='little',signed=True)
                    TlogPIDPitch[iPIDPitch,1] = int.from_bytes(char1, byteorder='little',signed=True)
                    TlogPIDPitch[iPIDPitch,2] = int.from_bytes(char2, byteorder='little',signed=True)
                    TlogPIDPitch[iPIDPitch,3] = int.from_bytes(char3, byteorder='little',signed=True)
                    TlogPIDPitch[iPIDPitch,4] = int.from_bytes(char4, byteorder='little',signed=True)
                    TlogPIDPitch[iPIDPitch,5] = int.from_bytes(char5, byteorder='little',signed=True)
                    TlogPIDPitch[iPIDPitch,6] = int.from_bytes(char6, byteorder='little',signed=True)
                    TlogPIDPitch[iPIDPitch,7] = int.from_bytes(char7, byteorder='little',signed=True)
                    iPIDPitch = iPIDPitch +1
                elif (ord(char0) == 2):                  #Yaw
                    TlogPIDYaw[iPIDYaw,0] = int.from_bytes(fourbytes, byteorder='little',signed=True)
                    TlogPIDYaw[iPIDYaw,1] = int.from_bytes(char1, byteorder='little',signed=True)
                    TlogPIDYaw[iPIDYaw,2] = int.from_bytes(char2, byteorder='little',signed=True)
                    TlogPIDYaw[iPIDYaw,3] = int.from_bytes(char3, byteorder='little',signed=True)
                    TlogPIDYaw[iPIDYaw,4] = int.from_bytes(char4, byteorder='little',signed=True)
                    TlogPIDYaw[iPIDYaw,5] = int.from_bytes(char5, byteorder='little',signed=True)
                    TlogPIDYaw[iPIDYaw,6] = int.from_bytes(char6, byteorder='little',signed=True)
                    TlogPIDYaw[iPIDYaw,7] = int.from_bytes(char7, byteorder='little',signed=True)
                    iPIDYaw = iPIDYaw +1                  
                    

              

 

print("************************* End log ****************************************")
print("Max Tick: ",MaxTick)
print("count PID: ",iPID)
print("count Max sample Time: ",iMaxsampleTime)
print("count PID Roll: ",iPIDRoll)
print("count PID Pitch: ",iPIDPitch)
print("count PID Yaw: ",iPIDYaw)
print("count Motor: ",iMotor)



print("TlogPIDPitch[0:20,0]: ",TlogPIDPitch[0:20,0])
print("TlogPIDPitch[0:20,1]: ",TlogPIDPitch[0:20,1])
print("TlogPIDPitch[0:20,2]: ",TlogPIDPitch[0:20,2])
print("TlogPIDPitch[0:20,3]: ",TlogPIDPitch[0:20,3])
print("TlogPIDPitch[0:20,4]: ",TlogPIDPitch[0:20,4])
print("TlogPIDPitch[0:20,5]: ",TlogPIDPitch[0:20,5])
print("TlogPIDPitch[0:20,6]: ",TlogPIDPitch[0:20,6])

print("TlogMotor[0:20,0]: ",TlogMotor[0:20,0])
print("TlogMotor[0:20,1]: ",TlogMotor[0:20,1])
print("TlogMotor[0:20,2]: ",TlogMotor[0:20,2])
print("TlogMotor[0:20,3]: ",TlogMotor[0:20,3])
print("TlogMotor[0:20,4]: ",TlogMotor[0:20,4])
print("TlogMotor[0:20,5]: ",TlogMotor[0:20,5])


plt.title('Roll')
plt.plot(TlogPIDRoll[0:iPIDRoll,0], TlogPIDRoll[0:iPIDRoll,1],'k*-' ,label='Roll',linewidth=1,markersize=1)
plt.plot(TlogPIDRoll[0:iPIDRoll,0], TlogPIDRoll[0:iPIDRoll,6],'r.-' ,label='RollPID',linewidth=1,markersize=1)
plt.legend()
plt.show()
plt.title('Pitch')
plt.plot(TlogPIDPitch[0:iPIDPitch,0],TlogPIDPitch[0:iPIDPitch,1],'k*-',label='Pitch',linewidth=1,markersize=1)
plt.plot(TlogPIDPitch[0:iPIDPitch,0],TlogPIDPitch[0:iPIDPitch,6],'g.-',label='PitchPID',linewidth=1,markersize=1)
plt.legend()
plt.show()
plt.title('Yaw')
plt.plot(TlogPIDYaw[0:iPIDYaw,0], TlogPIDYaw[0:iPIDYaw,1],'k*-',label='Yaw',linewidth=1,markersize=1)
plt.plot(TlogPIDYaw[0:iPIDYaw,0], TlogPIDYaw[0:iPIDYaw,6],'b.-',label='YawPID',linewidth=1,markersize=1)
plt.legend()
plt.show()

plt.title('Motors')
plt.plot(TlogMotor[0:iMotor,0],TlogMotor[0:iMotor,1],'k+-',label='Throttle',linewidth=1,markersize=1)
plt.plot(TlogMotor[0:iMotor,0],TlogMotor[0:iMotor,2],'r+',label='Front Left',linewidth=1,markersize=1)
plt.plot(TlogMotor[0:iMotor,0],TlogMotor[0:iMotor,3],'g+',label='Front Right',linewidth=1,markersize=1)
plt.plot(TlogMotor[0:iMotor,0],TlogMotor[0:iMotor,4],'b+',label='Rear Right',linewidth=1,markersize=1)
plt.plot(TlogMotor[0:iMotor,0],TlogMotor[0:iMotor,5],'y+',label='Rear Left',linewidth=1,markersize=1)
plt.legend()
plt.show()

fichier.close()
