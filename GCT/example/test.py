import sys
import numpy as np
import matplotlib.pyplot as plt


def on_press(event):
    print('press', event.key)
    sys.stdout.flush()
    if event.key == 'x':
        # visible = xl.get_visible()
        # xl.set_visible(not visible)
        # fig.canvas.draw()
        plt.close()
        # plt.disconnect(temp_id)

def on_press2(event):
    print('press', event.key)
    sys.stdout.flush()
    if event.key == 'enter':
        # visible = xl.get_visible()
        # xl.set_visible(not visible)
        # fig.canvas.draw()
        print('close')
        plt.close()
        # plt.disconnect(temp_id)

# Fixing random state for reproducibility
np.random.seed(19680801)

fig, ax = plt.subplots()

temp_id = plt.connect('key_press_event', on_press)
temp_id2 = plt.connect('key_press_event', on_press2)

ax.plot(np.random.rand(12), np.random.rand(12), 'go')
xl = ax.set_xlabel('easy come, easy go')
ax.set_title('Press a key')
plt.show()