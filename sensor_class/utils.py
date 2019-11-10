import matplotlib.pyplot as plt
import numpy as np

# robot_base = 10
counter = 0

def draw_warn_zone(fig, robot):
    dz = plt.Circle((robot.x_base, robot.y_base), robot.range, color='red', fill=False)
    fig.add_artist(dz)
    wz = robot.range + 1
    wz = plt.Circle((robot.x_base, robot.y_base), wz, color='orange', fill=False)
    fig.add_artist(wz)


def generate_word(human, robot, axes):
    axes.text(-8, 8, 'Generate World', fontsize=12)
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


def demo_sensor(demo_type, save_flag=False):
    from sensor_class.fence import CFence
    from sensor_class.human import CHuman
    from sensor_class.lidar import CLidar
    from sensor_class.mat import CMate
    import os
    lidar = CLidar(x=3, y=-2)
    mat = CMate('Mat1', x=-5, y=2, width=4, length=6)
    fence = CFence('Fence1', x=-4, y=1, length=4)
    human = CHuman('Worker1', start_vel=2, start_pos_x=-1, start_pos_y=2, heading=np.pi)
    if demo_type == "lidar":
        sensor = lidar
    elif demo_type == "mat":
        sensor = mat
    else:
        sensor = fence
    axes, fig = set_plot()
    current_time = 0
    dt = 0.010  # time step
    simulation_time = 3
    counter = 0
    cwd = os.getcwd()

    while current_time < simulation_time:
        if current_time == 0:
            for i in range(20):
                sensor.visualization(axes)
                plt.pause(0.01)
                plt.show()
                if save_flag:
                    plt.savefig(f'{cwd}/out/{demo_type}/fig{counter}.png')
                    counter += 1
        axes.cla()
        axes.set_xlim([-10, 10])
        axes.set_ylim([-10, 10])
        # axes.axis("equal")

        current_time += dt
        if human:
            human.update(dt, omega=0.1)
            human.plot(axes)

        sensor.plot(axes)
        sensor.detection(human, axes)
        plt.pause(0.01)
        plt.show()
        if save_flag:
            plt.savefig(f'{cwd}/out/{demo_type}/fig{counter}.png')
            counter += 1


def image2video(demo_type):
    import cv2
    import glob
    import os
    cwd = os.getcwd()
    img_array = []
    a = glob.glob(f'{cwd}/out/{demo_type}/*.png')
    for counter in range(len(a)):
        img = cv2.imread(f'{cwd}/out/{demo_type}/fig{counter}.png')
        height, width, layers = img.shape
        image_size = (width, height)
        img_array.append(img)

    out = cv2.VideoWriter(f'{cwd}/out/video/{demo_type}_1.avi', cv2.VideoWriter_fourcc(*'DIVX'), 15, image_size)

    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()


def image2gif(demo_type):
    import imageio
    import glob
    import os
    cwd = os.getcwd()
    images = []
    a = glob.glob(f'{cwd}/out/{demo_type}/*.png')
    for counter in range(len(a)):
        images.append(imageio.imread(f'{cwd}/out/{demo_type}/fig{counter}.png'))
    imageio.mimsave(f'{cwd}/out/gif/{demo_type}_1.gif', images)


if __name__ == '__main__':
    demo_type = "scenario"
    # demo_sensor(demo_type, save_flag=True)
    image2video(demo_type)
    image2gif(demo_type)
