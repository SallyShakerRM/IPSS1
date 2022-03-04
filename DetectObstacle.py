# -*- coding: utf-8 -*-
"""
Created on Fri Jan 24 13:09:04 2022

@author: Sally
"""

import sim
import sys
import time


print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
#clientID = -1
if clientID!=-1:
    print ('Connected to remote API server')    
    left_error,motorLeft=sim.simxGetObjectHandle(clientID,'EPUCK_leftJoint',sim.simx_opmode_oneshot_wait)
    right_error,motorRight=sim.simxGetObjectHandle(clientID,'EPUCK_rightJoint',sim.simx_opmode_oneshot_wait)
    #proximity sensors
  #  camera_error,camera=sim.simxGetObjectHandle(clientID,'ePuck_camera',sim.simx_opmode_oneshot_wait)
    
   # PCB_error2,PCB2=sim.simxGetObjectHandle(clientID,'ePuck_PCB2',sim.simx_opmode_oneshot_wait)
    proxSens=[-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]
    for i in range(0, 18):
        proxSens[i]=sim.simxGetObjectHandle(clientID,'EPUCK_proxSensor'+str(i),sim.simx_opmode_oneshot_wait)
        
    vLeft=0.8 #vLeft+braitenbergL[i]*detect[i]
    vRight=0.8 #vRight+braitenbergR[i]*detect[i]
    
    
    sim.simxSetJointTargetVelocity(clientID,motorLeft,vLeft,sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID,motorRight,vRight,sim.simx_opmode_streaming)
     
    proxSensReads=[-1,-1,-1,-1,-1,-1,-1,-1,-1]
    t3 = t = time.time() #record the initial time
    inFlag=0
    outFlag =0
    step_count = 0
    while (time.time()-t)<3000000:  #run for 30000 seconds
       # cameraReads=sim.simxGetVisionSensorImage (clientID,camera,0,sim.simx_opmode_streaming)
       # cameraReads=sim.simxGetVisionSensorImage(clientID,camera,0,sim.simx_opmode_buffer)
        for i in range(0, 8):
            proxSensReads[i]=sim.simxReadProximitySensor(clientID,proxSens[i][1],sim.simx_opmode_streaming)
            proxSensReads[i]=sim.simxReadProximitySensor(clientID,proxSens[i][1],sim.simx_opmode_buffer)
        if  proxSensReads[2][1] == True or proxSensReads[3][1] == True or proxSensReads[4][1] == True:
            t3 = time.time()
            inFlag = inFlag+1
            while (time.time()-t3)<5.5:
                sim.simxSetJointTargetVelocity(clientID,motorLeft,0.8,sim.simx_opmode_streaming)
                sim.simxSetJointTargetVelocity(clientID,motorRight,0.0,sim.simx_opmode_streaming)  
        else:
            # tt = outFlag
            # tt = tt+1
             if inFlag > outFlag:
                 for i in range(0, 8):
                     proxSensReads[i]=sim.simxReadProximitySensor(clientID,proxSens[i][1],sim.simx_opmode_streaming)
                     proxSensReads[i]=sim.simxReadProximitySensor(clientID,proxSens[i][1],sim.simx_opmode_buffer)
                 
                 while proxSensReads[1][1] == True or proxSensReads[0][1] == True:                      
                     sim.simxSetJointTargetVelocity(clientID,motorLeft,vLeft,sim.simx_opmode_streaming)
                     sim.simxSetJointTargetVelocity(clientID,motorRight,vRight,sim.simx_opmode_streaming)
                     step_count = step_count+2
                     for i in range(0, 8):
                         proxSensReads[i]=sim.simxReadProximitySensor(clientID,proxSens[i][1],sim.simx_opmode_streaming)
                         proxSensReads[i]=sim.simxReadProximitySensor(clientID,proxSens[i][1],sim.simx_opmode_buffer)
                 t2 = time.time()
                 while (time.time()-t2)<3:
                      sim.simxSetJointTargetVelocity(clientID,motorLeft,vLeft,sim.simx_opmode_streaming)
                      sim.simxSetJointTargetVelocity(clientID,motorRight,vRight,sim.simx_opmode_streaming)
                     
                 t2 = time.time()
                 while (time.time()-t2)<5:
                     sim.simxSetJointTargetVelocity(clientID,motorLeft,0.0,sim.simx_opmode_streaming)
                     sim.simxSetJointTargetVelocity(clientID,motorRight,0.8,sim.simx_opmode_streaming)
                 outFlag = 0
                 inFlag = 0
                 t2 = time.time()
                 while (time.time()-t2)<6:
                      sim.simxSetJointTargetVelocity(clientID,motorLeft,vLeft,sim.simx_opmode_streaming)
                      sim.simxSetJointTargetVelocity(clientID,motorRight,vRight,sim.simx_opmode_streaming)
                
                 proxSensReads[i]=sim.simxReadProximitySensor(clientID,proxSens[i][1],sim.simx_opmode_streaming)
                 proxSensReads[i]=sim.simxReadProximitySensor(clientID,proxSens[i][1],sim.simx_opmode_buffer)
                
                 while proxSensReads[1][1] == True or proxSensReads[0][1] == True:                     
                     sim.simxSetJointTargetVelocity(clientID,motorLeft,vLeft,sim.simx_opmode_streaming)
                     sim.simxSetJointTargetVelocity(clientID,motorRight,vRight,sim.simx_opmode_streaming)
                     for i in range(0, 8):
                         proxSensReads[i]=sim.simxReadProximitySensor(clientID,proxSens[i][1],sim.simx_opmode_streaming)
                         proxSensReads[i]=sim.simxReadProximitySensor(clientID,proxSens[i][1],sim.simx_opmode_buffer)
                # sim.simxSetJointTargetVelocity(clientID,motorLeft,vLeft,sim.simx_opmode_streaming)
                # sim.simxSetJointTargetVelocity(clientID,motorRight,vRight,sim.simx_opmode_streaming)
                 t2 = time.time()
                 while (time.time()-t2)<5:
                     sim.simxSetJointTargetVelocity(clientID,motorLeft,0.0,sim.simx_opmode_streaming)
                     sim.simxSetJointTargetVelocity(clientID,motorRight,0.8,sim.simx_opmode_streaming)
                # while step_count > 0:
                 t2 = time.time()
                 while (time.time()-t2)<6:
                     sim.simxSetJointTargetVelocity(clientID,motorLeft,vLeft,sim.simx_opmode_streaming)
                     sim.simxSetJointTargetVelocity(clientID,motorRight,vRight,sim.simx_opmode_streaming)
                     # step_count =step_count -1
                 t2 = time.time()
                 while (time.time()-t2)<5:
                     sim.simxSetJointTargetVelocity(clientID,motorLeft,0.8,sim.simx_opmode_streaming)
                     sim.simxSetJointTargetVelocity(clientID,motorRight,0.0,sim.simx_opmode_streaming)
               
             sim.simxSetJointTargetVelocity(clientID,motorLeft,vLeft,sim.simx_opmode_streaming)
             sim.simxSetJointTargetVelocity(clientID,motorRight,vRight,sim.simx_opmode_streaming)
    
   
else:
    print('Error in connection')
    sys.exit('cannot connect')
