from time import time

class Circle:

    def __init__(self, x, y, speed):

        self.init_x = x
        self.init_y = y
        self.speed =speed

        self.current_x = x
        self.current_y = y

        self.init_time = time()

    def __str__(self):
        return '(%s, %s)'%(self.current_x, self.current_y)

    def update_pos(self):

        time_difference = time() - self.init_time

        self.current_x = self.init_x - self.speed * time_difference
        self.current_y = self.init_y
    
    def is_equal(self, other_circle):

        if abs(self.current_x - other_circle.current_x) > 50 or abs(self.current_y - other_circle.current_y) > 50:
            return False
        else:
            return True
