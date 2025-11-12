# Author    : Killian Baillifard
# Date      : 12.11.2025
# Brief     : Non-linear control law

# Imports
import numpy as np
import robot_constants as constants
from matplotlib import pyplot as plt

# Control law functions
def sigmoid(x: float) -> float:
    """
    x -> y : R -> [0, 1]
    sigmoid(3) ~= 0.95
    sigmoid(-3) ~= 0.05
    """
    return 1 / (1 + np.exp(-x))

def control_law(error: float) -> float:
    SHIFT = 0.05
    SCALE = 100
    return constants.MAX_SPEED_MPS * (sigmoid(SCALE * (error - SHIFT)) - sigmoid(SCALE * (-error - SHIFT)))

if __name__ == '__main__':
    
    plt.figure('Control law')

    # Plot sigmoid function implementation
    x = np.linspace(-10, 10, 1000)
    y = sigmoid(x)
    plt.subplot(211)
    plt.title('Sigmoid function')
    plt.plot(x, y)
    plt.grid(True)
    plt.xlabel('x')
    plt.ylabel('y')

    # Plot control law function implementation
    error = np.linspace(-0.2, 0.2, 1000)
    speed = control_law(error)
    plt.subplot(212)
    plt.title('Control law')
    plt.plot(error, speed)
    plt.grid(True)
    plt.xlabel('error / m')
    plt.ylabel('speed / m/s')

    plt.tight_layout()
    plt.show()
