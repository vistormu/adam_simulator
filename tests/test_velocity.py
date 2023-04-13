from adam_sim import Adam
from adam_sim.entities import Velocity, AdamInfo

from vclog import Logger


def main():
    adam: Adam = Adam()
    
    initial_info: AdamInfo = adam.load()
    
    while True:
        adam.render()
        
        adam.left_manipulator.set_velocity_to(Velocity(2.0, 0.0, 0.0, 0.0, 2.0, 0.0))
                
        info: AdamInfo = adam.step()
        
        Logger.plain(f'Velocity: {info.left_manipulator.velocity}', color='secondary_red', flush=True)


if __name__ == '__main__':
    main()
    