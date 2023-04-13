from adam_sim import Adam, DataManager
from adam_sim.entities import Configuration, Velocity, Acceleration, AdamInfo

from vclog import Logger


def main():
    adam: Adam = Adam()
    
    initial_info: AdamInfo = adam.load()
    
    configurations: list[Configuration] = DataManager.load_configurations('test')
    
    for configuration in configurations:
        adam.render()
        
        adam.left_manipulator.set_to(configuration)
        
        info: AdamInfo = adam.step()
        
        # Logger.plain(f'Configuration: {info.left_manipulator.configuration}', color='cyan')
        Logger.plain(f'Velocity: {info.left_manipulator.velocity}', color='secondary_red', flush=True)
        # Logger.plain(f'Acceleration: {info.left_manipulator.acceleration}', color='secondary_green', flush=True)


if __name__ == '__main__':
    main()
    