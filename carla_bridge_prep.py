import os
import sys
import carla
import carla
                 
def _get_map_name(map_full_name):

    if map_full_name is None:
        return None
    name_start_index = map_full_name.rfind("/")
    if name_start_index == -1:
        name_start_index = 0
    else:
        name_start_index = name_start_index + 1        

    return map_full_name[name_start_index:len(map_full_name)]


def main():
    _local_host = os.environ['SIMULATOR_LOCAL_HOST']
    _port = int(os.environ['SIMULATOR_PORT'])
    _world = None

    try:
        client = carla.Client(_local_host, _port)
        client.set_timeout(20)    

        _world = client.get_world()
        town_map_name = _get_map_name(_world.get_map().name)

        print('export CARLA_TOWN_MAP="' + town_map_name + '"')
    except Exception as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()
