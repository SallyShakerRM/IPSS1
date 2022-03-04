
try:
    import sim
except:

import sys
import ctypes
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    # 1. First send a command to display a specific message in a dialog box:
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'displayText_function',[],[],['Hello world!'],emptyBuff,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Return string: ',retStrings[0]) # display the reply from CoppeliaSim (in this case, just a string)
    else:
        print ('Remote function call failed')

    # 2. Now create a dummy object at coordinate 0.1,0.2,0.3 with name 'MyDummyName':
    res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'createDummy_function',[],[0.1,0.2,0.3],['MyDummyName'],emptyBuff,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Dummy handle: ',retInts[0]) # display the reply from CoppeliaSim (in this case, the handle of the created dummy)
    else:
        print ('Remote function call failed')

    # 3. Now send a code string to execute some random functions:
    code="local octreeHandle=simCreateOctree(0.5,0,1)\n" \
    "simInsertVoxelsIntoOctree(octreeHandle,0,{0.1,0.1,0.1},{255,0,255})\n" \
    "return 'done'"
    res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,"remoteApiCommandServer",sim.sim_scripttype_childscript,'executeCode_function',[],[],[code],emptyBuff,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Code execution returned: ',retStrings[0])
    else:
        print ('Remote function call failed')

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
