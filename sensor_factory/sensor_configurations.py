from sensor_factory.generator_sensors import CSensorFactory
from sensor_factory.space_control import CSpace_control
from sensor_factory.coverage_detector import CCoverageDetector
import copy

sensor_number_max = 20


def generation_reset(sensors, detector, sensor_parameters):
    sensors.clear()
    detector.reset()
    CSpace_control.reset()
    sensor_parameters.clear()


def get_sensor_configurations(configuration_number=100, dangerous_zone_radius=2):
    configuration_counter = 0
    sensor_counter = 0
    sensor_price = 0

    sensors = []
    sensor_configuration = {}
    sensor_configurations = []
    sensor_parameters = []
    detector = CCoverageDetector()
    CCoverageDetector.dangerous_zone_radius = dangerous_zone_radius

    while configuration_counter < configuration_number:
        sensor_names = CSensorFactory.sensor_name_gene(sensor_number_max)

        for sensor_name in sensor_names:
            parameters_ = CSensorFactory.sensor_dict(sensor_name)
            generate_ = CSensorFactory.create_sensor(sensor_name, parameters_)
            if CSpace_control.check(generate_):
                parameters_ = CSensorFactory.sensor_dict(sensor_name)
                generate_ = CSensorFactory.create_sensor(sensor_name, parameters_)
                bad_placement_flag = detector.cover_update(generate_)
                if not bad_placement_flag:
                    if sensor_counter < sensor_number_max and 0 in set(detector.coverage_dict.values()):
                        sensors.append(generate_)
                        detector.cover_update(sensors[-1])
                        CSpace_control.update(sensors[-1])
                        sensor_counter += 1
                        sensor_price += sensors[-1].price
                        sensor_parameters.append((sensor_name, parameters_))
                    else:
                        break
                else:
                    continue
            else:
                continue

        if 0 in set(detector.coverage_dict.values()):
            # print("restart the generation")
            generation_reset(sensors, detector, sensor_parameters)
            sensor_counter = 0
            sensor_price = 0
        else:
            configuration_counter += 1
            sensor_configuration['sensor number'] = copy.deepcopy(sensor_counter)
            sensor_configuration['price'] = copy.deepcopy(sensor_price)
            sensor_configuration['parameters'] = copy.deepcopy(sensor_parameters)
            sensor_configurations.append(copy.deepcopy(sensor_configuration))
            # print("find one")
            sensor_counter = 0
            sensor_price = 0
            generation_reset(sensors, detector, sensor_parameters)

    return sensor_configurations

# TODO Speedup the generation of the sensor
# TODO Using pickel to store the configurations