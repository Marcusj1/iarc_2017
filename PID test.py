from PID import PID
import time
testPID = PID(.5,0.1,0)
current = 5
count = 5
while count > 0:
    return_value = testPID.run(2, current)
    current += return_value
    print(return_value)
    print(str(current) + '\n')
    time.sleep(0.1)
    count -= 1