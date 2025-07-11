import paho.mqtt.client as mqtt
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import sys


client = mqtt.Client(client_id="slam_map", transport="tcp", reconnect_on_failure=True)

grid_len = 0
grid_d = []
grid_str = []


def on_message(client, userdata, message, properties=None):
    global grid_d, grid_len

    print("received message")

    grid_d.append(message.payload.decode())
    grid_len += 1

    if grid_len == 4:
        grid_len = 0
        grid_str = [int(x) for x in "".join(grid_d)]

        grid_d = []

        print(f"total map: {grid_str}")
        print(f"len map: {len(grid_str)}")

        if len(grid_str) == 400:
            grid_data[:, :] = np.array(grid_str).reshape(20, 20)
        


def on_connect(client, userdata, flags, rc):
    client.subscribe("/map/bot1")

    print("Connected to mqtt server")


grid_data = np.zeros((20, 20), dtype=int)
cmap = mcolors.ListedColormap(['white', 'gray', 'black', 'purple'])
bounds = [0, 1, 2, 3, 4]
norm = mcolors.BoundaryNorm(bounds, cmap.N)


plt.ion()
fig, ax = plt.subplots(figsize=(6, 6))
heatmap = ax.imshow(grid_data, cmap=cmap, norm=norm,  extent=[0, 19, 0, 19])

# Set equal aspect ratio so cells are square
ax.set_aspect('equal', adjustable='datalim')

plt.title("Live Slam map")
plt.tight_layout()
plt.colorbar(heatmap, ticks=[0, 1, 2, 3], label='Value')

# Show grid lines for each cell
ax.set_xticks(np.arange(-0.5, 20, 1), minor=True)
ax.set_yticks(np.arange(-0.5, 20, 1), minor=True)
ax.grid(which='minor', color='black', linestyle='-', linewidth=0.5)

# Turn off major ticks and labels if not needed
ax.tick_params(which='both', bottom=False, left=False, labelbottom=False, labelleft=False)


if __name__ == "__main__":
    try:
        client.loop_stop()
        client.disconnect()
        client.on_connect = on_connect
        client.on_message = on_message

        client.connect("145.24.223.53", 9000, 60)

        client.loop_start()

        plt.show()

        while True:
            heatmap.set_data(grid_data)
            plt.pause(0.1)

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        client.loop_stop()