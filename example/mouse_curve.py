from GCT.curve_generator import curve_generator
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':

    point_list = []

    cg = curve_generator(select_mode='mouse')

    curve = cg.generate_curve('dubins', point_list, 0.1, 1)

    if curve is not None: cg.plot_curve(curve, show_direction=False)

    plt.show()