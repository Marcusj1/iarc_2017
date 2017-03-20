from PIDClass import PID
from PID import PID
import time
testPID = PID(1.1,0.0,0.0, 100)
current = 0
count   = 20
while count > 0:
    return_value = testPID.run(2, current)
    current += return_value
    #print(return_value)
    print(str(current) )
    time.sleep(0.01)
    count -= 1