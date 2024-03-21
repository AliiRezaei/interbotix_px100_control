import numpy as np

class RotationMatrix:
    def __init__(self, theta, axis):
        self.theta = theta
        self.axis  = axis

        if self.axis == 'x':
            self.R = np.round(self.RotationX(), 4)
        elif self.axis == 'y':
            self.R = np.round(self.RotationY(), 4)
        elif self.axis == 'z':
            self.R = np.round(self.RotationZ(), 4)


    def RotationX(self):
        return np.matrix([[1,            0,                   0,             0],
                          [0,   np.cos(self.theta),   -np.sin(self.theta),   0],
                          [0,   np.sin(self.theta),    np.cos(self.theta),   0],
                          [0,            0,                   0,             1]])
    
    def RotationY(self):
        return np.matrix([[np.cos(self.theta),    0,   np.sin(self.theta),   0],
                          [        0,             1,              0,         0],
                          [-np.sin(self.theta),   0,   np.cos(self.theta),   0],
                          [        0,             0,              0,         1]])
    
    def RotationZ(self):
        return np.matrix([[np.cos(self.theta),   -np.sin(self.theta),     0,    0],
                          [np.sin(self.theta),    np.cos(self.theta),     0,    0],
                          [       0,                      0,              1,    0],
                          [       0,                      0,              0,    1]])


def main():
    theta = np.pi
    RotX = RotationMatrix(theta/4, 'x')
    RotY = RotationMatrix(theta/3, 'y')
    R = RotX.R @ RotY.R
    print(R)

if __name__ == '__main__':
    main()