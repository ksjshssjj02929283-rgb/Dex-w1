
#---------------------w1_ecat_topology.conf----------------
# #注意: index必须是递增的

# #name,index
# ANKLE,1
# KNEE,2
# BUTTOCK,3
# WAIST,4
# NECK1,7
# NECK2,8
# LEFT_J1,9
# LEFT_J2,10
# LEFT_J3,11
# LEFT_J4,12
# LEFT_J5,13
# LEFT_J6,14
# LEFT_J7,15
# RIGHT_J1,16
# RIGHT_J2,17
# RIGHT_J3,18
# RIGHT_J4,19
# RIGHT_J5,20
# RIGHT_J6,21
# RIGHT_J7,22
#----------------------------------------------------------

import os

def read_config_file(config_path):
    """
    Reads the configuration file and returns a dictionary with the motor names and their indices.
    """
    config = {}
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Config file {config_path} does not exist.")

    with open(config_path, 'r') as file:
        for line in file:
            if line.strip() and not line.startswith('#'):
                name, index = line.strip().split(',')
                config[name] = int(index)

    return config

if __name__ == '__main__':
    # Example usage
    config_path = '../config/w1_ecat_topology.conf'
    config = read_config_file(config_path)
    print(config)