
global count_old
count_old = [([0]*4) for i in range(20)]

class SPEED():
    def __init__(self, period, delay):
        self.timer_period = period
        self.count_delay = delay
        self.count_old = [([0]*4) for i in range(self.count_delay)]

    def speed_update(self, index, count_now=0):
        a = self.timer_period
        b = self.count_delay
        self.count_sub = count_now - self.count_old[0][index]
        if self.count_sub == 0:
            speed_now = 0
        else:
            speed_now = self.count_sub / (a / 32)

        for p in range(b - 1):
            self.count_old[p][index] = self.count_old[p + 1][index]
        p = 0

        self.count_old[b - 1][index] = count_now
        #print(self.count_old[0][0],self.count_old[0][1],self.count_old[0][2],self.count_old[0][3],self.count_sub)
        return speed_now
