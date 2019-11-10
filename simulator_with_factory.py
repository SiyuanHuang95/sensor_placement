from sensor_class.human import CHuman
from sensor_class.robot import CRobot
from sensor_factory.generator_sensors import CSensorFactory
from sensor_factory.sensor_configurations import get_sensor_configurations
from sensor_class.utils import *
import numpy as np


generate_world = False
configuration_number = 500
dangerous_zone_radius = 2
sensors = []

CSensorFactory.dangerous_zone_radius = dangerous_zone_radius
sensor_configurations = get_sensor_configurations(configuration_number, dangerous_zone_radius)
sensor_number_list = [sensor_configurations[i]['sensor number'] for i in range(len(sensor_configurations))]
minimum_number_configuration = sensor_number_list.index(min(sensor_number_list))

for i in range(len(sensor_configurations[minimum_number_configuration]['parameters'])):
    generate_ = CSensorFactory.create_sensor(sensor_configurations[8]['parameters'][i][0],
                                             sensor_configurations[8]['parameters'][i][1])
    sensors.append(generate_)

axes, fig = set_plot()
counter = 0

current_time = 0
dt = 0.010  # time step
simulation_time = 10

human = CHuman('Worker1', start_vel=5, start_pos_x=2.5, start_pos_y=3, heading=-np.pi / 1.2)
robot = CRobot(robot_range=dangerous_zone_radius, name='Robot1', start_vel=0.1, start_pos=0)


# save_animation = True
if not generate_world:
    generate_word(human=human, robot=robot, axes=axes)

    # if save_animation:
    #     plt.savefig(f'./out/fig{counter}.png')
    #     counter += 1
    generate_world = True
    #
    # if save_animation:
    #     plt.savefig(f'./out/fig{counter}.png')
    #     counter += 1

    for sensor in sensors:
        sensor.visualization(axes)
        plt.pause(1)
        plt.show()

while current_time < simulation_time:

    axes.cla()
    axes.set_xlim([-10, 10])
    axes.set_ylim([-10, 10])
    # axes.axis("equal")

    current_time += dt
    if human:
        human.update(dt, omega=0.1)
        human.plot(axes)
    robot.update()

    robot.plot(axes)
    draw_warn_zone(axes, robot)
    for sensor in sensors:
        sensor.plot(axes)
        sensor.detection(human, axes)
    plt.pause(0.01)
    plt.show()

    # if save_animation:
    #     plt.savefig(f'./out/fig{counter}.png')
    #     counter += 1
# TODO If needed, using multithreading to paralle the processing
