import signal
import os
import sys
import importlib
import time
import traceback
import carla
import threading
import json
import socket
import datetime
import math
import carla
import subprocess

class BridgeHelpers(object):
    @staticmethod
    def get_agent_actor(world, role_name):
        actors = world.get_actors().filter('*vehicle*')
        for car in actors:
            if car.attributes['role_name'] == role_name:
                return car
        return None

class AgentLoop(object):
    
    def __init__(self):
        self.start_game_time = None
        self.start_system_time = None
        self.debug_mode = False
        self.agent = None
        self.ego_vehicle = None
        self.running = False
        self.timestamp_last_run = 0.0
        self.timeout = 20.0
        self.role_name = os.environ['AGENT_ROLE_NAME']
        self.mode2_host = os.environ['SIMULATOR_LOCAL_HOST']
        self.mode2_port = os.environ.get('SIMULATOR_MODE2_PORT', None)
        
        self.frame_rate = float(os.environ['AGENT_FRAME_RATE'])
        self.timeout_max = self.frame_rate * 30

        self.time_out_allow = True
        self.current_timeout_count = 0

        self.spectator = None

        # Change port to int if it exists
        if self.mode2_port != None:
            self.mode2_port = int(self.mode2_port)

        ## Pasta
        self.current_ergo_action = None

        try:
            with open(os.path.join(sys.path[0],'CAN_ID.json'), 'r') as file:
                carlaData = file.read()
        except Exception as e:
            print("Failed to read file: " + os.path.join(sys.path[0],'CAN_ID.json'))
            quit()

        self.carlaIDMapCMD = self.load_CAN_ID('command', carlaData)
        self.carlaIDMapRPT = self.load_CAN_ID('report', carlaData)
        self.carlaFullSLCANMap = self.load_SLCANMap(carlaData)

        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pasta_send_thread = threading.Thread(target = self.handle_PASTA_sending, args = [self.send_sock, (self.mode2_host, self.mode2_port)])
        self.pasta_send_thread.setDaemon(True)
        # self.pasta_send_thread.start()

    def _stop_loop(self):
        self.running = False

        # if self.ego_vehicle is not None:
        #     print("Destroying ego vehicle")
        #     self.ego_vehicle.destroy()

        if self.pasta_send_thread.is_alive:
            print("Joining Pasta thread")
            self.pasta_send_thread.join()


    def _tick_agent(self, world):          
        snapshot = world.get_snapshot()
        if snapshot:
            timestamp = snapshot.timestamp
        else:
            timestamp = 0.0

        if self.spectator is None:
            self.spectator = world.get_spectator()

        if self.timestamp_last_run < timestamp.elapsed_seconds and self.running:
            self.timestamp_last_run = timestamp.elapsed_seconds
            
            self.current_ergo_action = self.agent.run_step(timestamp)

            #TODO Entry area for data passing
            # try:
            #     self.current_ergo_action = self.agent()
            
            # except SensorReceivedNoData as e:
            #     raise RuntimeError(e)

            # except Exception as e:
            #     raise AgentError(e)
 
            self.ego_vehicle = BridgeHelpers.get_agent_actor(world, self.role_name)

            if self.ego_vehicle is None:
                if self.time_out_allow == True:
                    self.current_timeout_count += 1
                    if self.current_timeout_count > self.timeout_max:
                        self.time_out_allow = False
                else:
                    self.running = False
            else:
                self.time_out_allow = False
                ego_trans = self.ego_vehicle.get_transform()
                ego_trans_forward = ego_trans.get_forward_vector()
                self.spectator.set_transform(carla.Transform(ego_trans.location + carla.Location(z=50) - carla.Location(x=(ego_trans_forward.x * 50), y=(ego_trans_forward.y * 50)),
                                                        carla.Rotation(pitch=-45, yaw=ego_trans.rotation.yaw)))

    def handle_PASTA_sending(self, sock, can_addr):
        if self.mode2_port == None:
            print("Mode2 is diabled")
            return

        print("Mode2 started on ", can_addr[0], ":", str(can_addr[1]))

        current_time = datetime.datetime.now()
        time_period = datetime.timedelta(milliseconds=50)
        goal = current_time + time_period
        prevHandBrake = 0
        lastIgnition = 0
        g_prev = None

        ## Wait for start of processing
        while (self.current_ergo_action == None) and self.running:
            time.sleep(0.02)

        if self.running == False:
            return

        ## setup previous tracking
        g_cur_control = self.current_ergo_action
        g_prev = g_cur_control

        ## Never not on in Autoware
        g_ignition_on = True

        ## Get Max Steer Angle
        carla_max_steer_angle = math.degrees(abs(self.current_ergo_action["info"].wheels[0].max_steer_angle))
        carla_max_rpm = self.current_ergo_action["info"].max_rpm

        while self.running:
            time.sleep(0.02)
            g_prev = g_cur_control
            g_cur_control = self.current_ergo_action
            current_time = datetime.datetime.now()
            send_50ms_period_msg = False

            wheel_angle = int(-g_cur_control["status"].control.steer * carla_max_steer_angle)

            engine_rpm = carla_max_rpm * g_cur_control["status"].control.throttle
            if engine_rpm > 10000:
                engine_rpm = 10000
            if engine_rpm < 600: 
                engine_rpm = 600
            if not g_ignition_on:
                engine_rpm = 0
            g_rpm_this_tick = int(engine_rpm)


            if current_time > goal:
                goal = current_time + time_period
                send_50ms_period_msg = True
            try:
                #global prev, 
                #prec = prev
                #rospy.loginfo(rospy.get_caller_id() +  " {:03x} {:0{datasize}x}".format(self.carlaIDMapCMD['throttle']  , (int) (g_cur_control.throttle * 0x3FF), datasize=2*self.carlaFullSLCANMap[self.carlaIDMapCMD['throttle']  ]['datasize']) )
                v    = g_cur_control["status"].velocity
                #kmh = int(3.6 * v)
                kmh = int(3.6 * v)
                # bll = g_lidar_min_br if g_lidar_min_br != LIDAR_BIG_VAL else 0
                # brl = g_lidar_min_bl if g_lidar_min_bl != LIDAR_BIG_VAL else 0
                # gear = 1 # P
                if g_ignition_on: #TODO hack but looks like autopilot doesn't update gear so just set it to D (and just in case let it update)
                    gear = 4
                if 0 < g_cur_control["status"].control.gear and g_cur_control["status"].control.gear <= 3:
                    gear = 5 # L
                elif g_cur_control["status"].control.gear > 3:
                    gear = 4 # D
                if g_cur_control["status"].control.reverse:
                    gear = 2 # R
                
                # autopilot doesn't touch lights just sending 0
                #pastaFrontLights, pastaTurnLights, pastaPassing = carla_to_pasta_light_map(g_current_lights)
                sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}".format(self.carlaIDMapCMD['throttle']  , (int) (g_cur_control["status"].control.throttle * 0x3FF)               , datasize=2*self.carlaFullSLCANMap[self.carlaIDMapCMD['throttle']  ]['datasize']), 'ascii'), can_addr)
                sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}".format(self.carlaIDMapCMD['steer']     , (int) (self.carla_to_pasta_steer_map(g_cur_control["status"].control.steer)), datasize=2*self.carlaFullSLCANMap[self.carlaIDMapCMD['steer']     ]['datasize']), 'ascii'), can_addr)
                sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}".format(self.carlaIDMapCMD['brake']     , (int) (g_cur_control["status"].control.brake * 0x3FF)                  , datasize=2*self.carlaFullSLCANMap[self.carlaIDMapCMD['brake']     ]['datasize']), 'ascii'), can_addr)
                # TODO no up down so not sending
                #sock.sendto(bytes("{:03x} {:0{datasize}x}".format(self.carlaIDMapCMD['gear']      , (int) (g_ger_control)                                , datasize=2*self.carlaFullSLCANMap[self.carlaIDMapCMD['gear']      ]['datasize']), 'ascii'), can_addr)
                sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}".format(self.carlaIDMapCMD['lightTurn'] , 0 , datasize=2*self.carlaFullSLCANMap[self.carlaIDMapCMD['lightTurn'] ]['datasize']), 'ascii'), can_addr)
                # in mode 2 we are always going to be applying gear so we need to change carla gear here + send external gear
                sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}".format(self.carlaIDMapRPT['gear'],  gear, datasize=2*self.carlaFullSLCANMap[self.carlaIDMapRPT['gear']]['datasize']), 'ascii'), can_addr) 
                if send_50ms_period_msg:
                    sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}".format(self.carlaIDMapCMD['hand_brake'], (int) (g_cur_control["status"].control.hand_brake)                     , datasize=2*self.carlaFullSLCANMap[self.carlaIDMapCMD['hand_brake']]['datasize']), 'ascii'), can_addr)
                    sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}".format(self.carlaIDMapCMD['lightFront'], 0, datasize=2*self.carlaFullSLCANMap[self.carlaIDMapCMD['lightFront']]['datasize']), 'ascii'), can_addr)
                    sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}".format(self.carlaIDMapCMD['passing']   , 0, datasize=2*self.carlaFullSLCANMap[self.carlaIDMapCMD['passing']   ]['datasize']), 'ascii'), can_addr)
                    # might need better word for this RPT hand_brake is more of a switch not exact state like CMD
                    sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}".format(self.carlaIDMapRPT['hand_brake'], (int)(g_cur_control["status"].control.hand_brake), datasize=2*self.carlaFullSLCANMap[self.carlaIDMapRPT['hand_brake']]['datasize']), 'ascii'), can_addr)
                sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}"               .format(self.carlaIDMapRPT['brake']     , (int) (g_prev["status"].control.brake    * 0x3FF)       , datasize=2*self.carlaFullSLCANMap[self.carlaIDMapRPT['brake']      ]['datasize']), 'ascii'), can_addr)
                sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}"               .format(self.carlaIDMapRPT['throttle']  , (int) (g_prev["status"].control.throttle * 0x3FF)       , datasize=2*self.carlaFullSLCANMap[self.carlaIDMapRPT['throttle']   ]['datasize']), 'ascii'), can_addr)
                ## not 2x data size as has 2 variable worth so half for each
                sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}{:0{datasize}x}".format(self.carlaIDMapRPT['rpm']       , g_rpm_this_tick, kmh                     , datasize=  self.carlaFullSLCANMap[self.carlaIDMapRPT['rpm']        ]['datasize']), 'ascii'), can_addr)
                ## TODO 0 for torque
                sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}0000"           .format(self.carlaIDMapRPT['steer']     , self.carla_to_pasta_steer_map(g_prev["status"].control.steer), datasize=2*self.carlaFullSLCANMap[self.carlaIDMapRPT['steer']      ]['datasize']), 'ascii'), can_addr)
                sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}"               .format(self.carlaIDMapRPT['tire_angle'], wheel_angle, datasize=2*self.carlaFullSLCANMap[self.carlaIDMapRPT['tire_angle'] ]['datasize']), 'ascii'), can_addr)
                # sock.sendto(bytes("{:03x} {:0{datasize}x}{:0{datasize}x}".format(self.carlaIDMapRPT['lidar']     , bll, brl                              , datasize=self.carlaFullSLCANMap[self.carlaIDMapRPT['lidar'] ]['datasize']), 'ascii'), can_addr)
                if send_50ms_period_msg:
                    sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}"               .format(self.carlaIDMapRPT['kmh']       , kmh, datasize=2*self.carlaFullSLCANMap[self.carlaIDMapRPT['kmh']   ]['datasize']), 'ascii'), can_addr)
                    sock.sendto(bytes("lf6-{:03x} {:0{datasize}x}"               .format(self.carlaIDMapRPT['engine']    , g_ignition_on, datasize=2*self.carlaFullSLCANMap[self.carlaIDMapRPT['engine']]['datasize']), 'ascii'), can_addr)
            except Exception:
                print("PASTA send thread exception")
                # print(e)

    def load_CAN_ID(self, levelToUse, data, field='key'):
        carlaIDMap = {}
        initDict = json.loads(data)
        candIDsDict = initDict['can_id']
        # try and find IDs witha not None carlaVar value for a given ID
        for key, val in candIDsDict.items():
            try:
                carlaVar = val['carlaVar']
                canLevel = val['level']
                if carlaVar is not None and canLevel == levelToUse:
                    if field == 'key':
                        carlaIDMap[carlaVar] = int(key)
                    else:
                        carlaIDMap[carlaVar] = val[field]
            except Exception as e:
                print("Failed to parse CAN_ID json file")
                print(e)
                print("key, val", key, ",", val)
        return carlaIDMap

    def load_SLCANMap(self, data):
        output = {}
        jsonData = json.loads(data)['can_id']
        for k, v in jsonData.items():
            output[int(k)] = v

        return output

    def carla_to_pasta_steer_map(self, steer):
        pastaSteer = (int)(steer * 0x1FF)
        if (pastaSteer & (1 << (16 - 1))) != 0:
            pastaSteer += 1<<16
        return pastaSteer



class WorldHandler(object):
    def __init__(self):
        self._local_host = os.environ['SIMULATOR_LOCAL_HOST']
        self._port = int(os.environ['SIMULATOR_PORT'])
        self._frame_rate = float(os.environ['AGENT_FRAME_RATE'])
        self._agent_role_name = os.environ['AGENT_ROLE_NAME']
        self._bridge_mode = os.environ['OP_BRIDGE_MODE']
        self._map_name = os.environ['FREE_MAP_NAME']
        self._spawn_point = os.environ['FREE_AGENT_POSE']
        self._world = None
    
    def load_world(self):
        client = carla.Client(self._local_host, self._port)
        client.set_timeout(20)    
  
        self._world = client.get_world()
        if self._world is not None:
            settings = self._world.get_settings()
            settings.fixed_delta_seconds = 1.0 / self._frame_rate
            settings.synchronous_mode = True
            self._world.apply_settings(settings)   

            self._world.tick()     
     
            # spawn_point = carla.Transform()
            # point_items = self._spawn_point.split(',')
            # _randomize = False
            # if len(point_items) == 6:
            #     spawn_point.location.x = float(point_items[0])
            #     spawn_point.location.y = float(point_items[1])
            #     spawn_point.location.z = float(point_items[2]) + 2  
            #     spawn_point.rotation.roll = float(point_items[3])
            #     spawn_point.rotation.pitch = float(point_items[4])
            #     spawn_point.rotation.yaw = float(point_items[5])
            # else:
            #     _randomize = True                
        
            # CarlaDataProvider.request_new_actor('vehicle.toyota.prius', spawn_point, self._agent_role_name, random_location=_randomize)            
        else:
            print("Can't Load CARLA .. make sure that Simulator is running !!")  

    def get_world(self):
        return self._world

    def _cleanup(self):
        settings = self._world.get_settings()
        settings.fixed_delta_seconds = 1.0 / self._frame_rate
        settings.synchronous_mode = False
        self._world.apply_settings(settings)   

        # CarlaDataProvider.cleanup()                    

class AgentHandler(object):
    def __init__(self, world_handler):
        self._agent_role_name = os.environ['AGENT_ROLE_NAME']
        self._frame_rate = float(os.environ['AGENT_FRAME_RATE'])
        agent_path = os.environ['TEAM_AGENT']    
        module_name = os.path.basename(agent_path).split('.')[0]    
        sys.path.insert(0, os.path.dirname(agent_path))    
        module_agent = importlib.import_module(module_name)    
        agent_class_name = getattr(module_agent, 'get_entry_point')()
        self.agent_instance = getattr(module_agent, agent_class_name)(world_handler)
        self.world_handler = world_handler
    
    def run_agent(self):
        try:    
            self.agent_loop = AgentLoop()
            self.agent_loop.role_name = self._agent_role_name            
            self.agent_loop.running = True  
            self.agent_loop.agent = self.agent_instance
            self.loopLock = threading.Event()
            self.loopTimer = threading.Thread(None, self.timer_tick, "AgentLoopTimer")  
            self.loopTimer.start()

            #pasta
            self.agent_loop.pasta_send_thread.start()

            world = self.world_handler.get_world()
            if world:
                while self.agent_loop.running:
                    world.tick()
                    self.agent_loop._tick_agent(world)  
                    if self.loopLock.wait(2.0):
                        self.loopLock.clear() 

                                
        except Exception as e:        
            traceback.print_exc()
    
    def timer_tick(self):
        try:
            while self.agent_loop.running:
                time.sleep(1.0 / self._frame_rate)
                self.loopLock.set()
            self.loopLock.clear() 
        finally:
            self.loopLock.clear() 
            return

    def _stop_loop(self, sig_num, frame):
        self.agent_loop._stop_loop()
        self.loopTimer.join()
    
    def _cleanup(self):
        if self.agent_instance:                
            self.agent_instance.destroy()
            self.agent_instance = None

        if  self.agent_loop:
             self.agent_loop = None

def main():
    world_handler = WorldHandler()
    world_handler.load_world()      
    
    agent_handler = AgentHandler(world_handler)

    signal.signal(signal.SIGINT, agent_handler._stop_loop)

    agent_handler.run_agent()

    print("Scenario Ended , Hero has fallen, Sayonara")  

    agent_handler._cleanup()
    world_handler._cleanup()

    print("Terminating possible rviz2 process")
    rviz_ps = subprocess.Popen(['ps'], stdout=subprocess.PIPE)
    ps_out, ps_err = rviz_ps.communicate()
    for line in ps_out.splitlines():
        if 'rviz2' in str(line):
            pid = int(line.split(None, 1)[0])
            os.kill(pid, signal.SIGKILL)
            print("   Killed %i" % pid)

if __name__ == '__main__':
    main()
