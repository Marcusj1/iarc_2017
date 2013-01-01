import time
class PID:
    def __init__(self, KP, KI, KD, maximum_magnitude):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.maximum_magnitude = maximum_magnitude
        self.previous_error = 0
        self.integral = 0
        self.old_time = time.time()
        self.goal = 987


    def run(self, goal, current):
        dt = time.time() - self.old_time
        self.old_time = time.time()
        error = goal - current

        # Sets the  integral
        self.integral += error * dt
        # Sets the derivative

        derivative = (error - self.previous_error) / dt
        if not self.goal == goal:
            derivative = 0
            self.goal = goal

        # Adjusts the output by the constants P, I, and D
        print(str(error) + '  |  ' + str(derivative ) + '  |  ' + str(self.integral))
        output = ( self.KP * error ) + ( self.KI * self.integral ) + ( self.KD * derivative )

        self.magnitude_limits(output, self.maximum_magnitude)

        # Sets the previous error
        self.previous_error = error

        return output

    def magnitude_limits(self, output, maximum_magnitude):
        if output > maximum_magnitude:
            output = maximum_magnitude
        elif output < - maximum_magnitude:
            output = -maximum_magnitude
        else:
            output = output
        return output