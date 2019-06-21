import matplotlib.pyplot as plt


def draw_warn_zone(fig, robot, wz=3):
    dz = plt.Circle((robot.x_base, robot.y_base), robot.range, color='red', fill=False)
    fig.add_artist(dz)
    wz = plt.Circle((robot.x_base, robot.y_base), wz, color='orange', fill=False)
    fig.add_artist(wz)

