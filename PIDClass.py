import time
class PID:
    def __init__(self, KP, KI, KD, maximum_magnitude, maximum_integral):
        print("KP:" + str(KP) + " KI:" + str(KI) + " KD:" + str(KD))
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.maximum_magnitude = maximum_magnitude
        print(maximum_magnitude)
        print(maximum_integral)
        print("-----------")
        self.maximum_integral  = maximum_integral
        self.previous_error = 0
        self.integral = 0
        self.old_time = time.time()
        self.goal = 987
	self.pervious_error = 0

    def run(self, goal, current):
        dt = time.time() - self.old_time
        self.old_time = time.time()
        error = goal - current

        # Sets the  integral
        self.integral += error * dt
        self.integral = self.magnitude_limits( self.integral, self.maximum_integral)


        if dt > 1: #or abs(self.pervious_error - error) > 0.19:
            self.integral = 0
            print("Reset I portion")

        # Sets the derivative
        derivative = (error - self.previous_error) / dt
        if not self.goal == goal:
            derivative = 0
            self.goal = goal

        # Adjusts the output by the constants P, I, and D
        #print("                                    " + str(error) + '  |  ' + str(self.integral ) + '  |  ' + str(derivative))
        print("                                    " + str(error*self.KP) + '  |  ' + str(self.integral*self.KI) + '  |  ' + str(derivative*self.KD))
        output = ( self.KP * error ) + ( self.KI * self.integral ) + ( self.KD * derivative )
        output = self.magnitude_limits(output, self.maximum_magnitude)

        # Sets the previous error
        self.previous_error = error
        return output

    def magnitude_limits(self, output, maximum_magnitude):
        if output > maximum_magnitude:
            output = maximum_magnitude
        elif output < - maximum_magnitude:
            output = -maximum_magnitude
        return output
