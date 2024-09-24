import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Load the CSV file into a Pandas DataFrame
filename = 'data_2023-10-20_16-48-56'
df = pd.read_csv('Outputs/' + filename + '.csv', index_col=0, parse_dates=True, header=0)

data = np.array(df)

# Create a figure and axis object
fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 8))

# Define a function to update the plot at each timestep
def update_plot(i):
    r0 = 2
    radii = data[i, 1:]*.1 + r0
    radii = np.append(radii, radii[0])
    n = np.arange(0, 21, 1)

    x = -radii * np.sin(-n * 2 * np.pi / 20)
    y = -radii * np.cos(-n * 2 * np.pi / 20)

    ax.clear()
    ax.plot(x, y)

    n = np.arange(0, 210, 1) / 10.
    ax.plot(r0 * np.cos(n * 2 * np.pi / 20), r0 * np.sin(n * 2 * np.pi / 20), 'k', linewidth=.5)
    ax.plot(1.5*r0 * np.cos(n * 2 * np.pi / 20), 1.5*r0 * np.sin(n * 2 * np.pi / 20), 'k:', linewidth=.5)
    ax.plot(2*r0 * np.cos(n * 2 * np.pi / 20), 2*r0 * np.sin(n * 2 * np.pi / 20), 'k--', linewidth=.5)
    ax.plot(2.5 * r0 * np.cos(n * 2 * np.pi / 20), 2.5 * r0 * np.sin(n * 2 * np.pi / 20), 'k:', linewidth=.5)
    ax.plot(3 * r0 * np.cos(n * 2 * np.pi / 20), 3 * r0 * np.sin(n * 2 * np.pi / 20), 'k--', linewidth=.5)

    ax.set_xlim([-3.5*r0, 3.5*r0])
    ax.set_ylim([-3.5 * r0, 3.5 * r0])
    ax.set_xlabel('Force [u]')
    ax.set_title('Forces on object')
    ax.set_aspect('equal', 'box')

    ax2.clear()
    ax2.plot(df.index[:i], data[:i, 0])
    ax2.grid()
    ax2.set_ylim([-1, 10])

    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Grasp Metric')
    ax2.set_title('Time Evolution of Data')


# Create an animation object
ani = animation.FuncAnimation(fig, update_plot, frames=len(df), interval=120)

# Save the animation as a video file
# Set up formatting for the movie files
ani.save('Outputs/' + 'Out_Rev_' + filename + '.mp4')

# Show the interactive plot
plt.show()
