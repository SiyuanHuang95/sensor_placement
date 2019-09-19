import matplotlib.pyplot as plt


def draw_warn_zone(fig, robot, wz=3):
    dz = plt.Circle((robot.x_base, robot.y_base), robot.range, color='red', fill=False)
    fig.add_artist(dz)
    wz = plt.Circle((robot.x_base, robot.y_base), wz, color='orange', fill=False)
    fig.add_artist(wz)


def generate_word(human, robot, axes):
    axes.text(-8,  8, 'Generate World', fontsize=12)
    human.plot(axes)
    plt.pause(1)
    plt.show()
    robot.plot(axes)
    plt.pause(1)
    plt.show()


def set_plot():
    fig, axes = plt.subplots(1, 1)
    plt.ion()
    # axes = plt.gca()
    axes.set_xlim([-10, 10])
    axes.set_ylim([-10, 10])
    return axes, fig

