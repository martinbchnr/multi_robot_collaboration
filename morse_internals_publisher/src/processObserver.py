#!/usr/bin/env python

import psutil
import time
import rosnode
import rospy
import json
import signal
import logging
import sys
import os
from enum import Enum
from datetime import datetime
try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy
from socket import error as SocketError
import errno

ID = '/NODEINFO'
#OUT_SUFFIX = datetime.now().strftime('%Y-%m-%d-%H-%M')
OUT_FILE_NAME = 'perfdata_'
BASE_DIR = os.path.join(os.path.dirname(__file__), 'perfData') #Adapt accordingly or output will be saved in package src dir
SIMULATOR_NODE_NAME = "/morse"

class Granularity(Enum):
    TICKED = 1
    AVERAGED = 2

class RecordOption(Enum):
    ALL = 1
    NODE_STATS_ONLY = 2 #Only ROS processes supplied by master
    OVERALL_ONLY = 3 #Whole system

class TerminationHandler:
  kill_now = False
  def __init__(self):
    signal.signal(signal.SIGINT, self.set_termination_flag)
    signal.signal(signal.SIGTERM, self.set_termination_flag)

  def set_termination_flag(self,signum, frame):
    self.kill_now = True

class SystemObserver():
    nodes = dict()
    termination_handler = TerminationHandler()
    start_time = round(time.time())
    io_writes_baseline = psutil.disk_io_counters().write_bytes

    def first_time_call(self):
        self.overall_stats()
        self.get_process_stats()

    def overall_stats(self):
        overall_stat = dict()
        overall_stat['cpu_stats'] = {"cpu_percent_percpu": psutil.cpu_percent(interval=None,percpu=True), "cpu_percent": psutil.cpu_percent(interval=None,percpu=False)}
        overall_stat['mem_stats'] = psutil.virtual_memory()._asdict()
        overall_stat['io_stats'] = psutil.disk_io_counters()._asdict()
        return overall_stat

    def get_all_node_info(self):
        # related functions stem from rqt_top
        infos = []
        self.remove_dead_nodes()
        for node_name in rosnode.get_node_names():
            info = self.get_node_info(node_name)
            if info is not False: infos.append((node_name, info))
        return infos

    def remove_dead_nodes(self):
        running_nodes = rosnode.get_node_names()
        dead_nodes = [node_name for node_name in self.nodes if node_name not in running_nodes]
        for node_name in dead_nodes:
            self.nodes.pop(node_name, None)

    def get_node_info(self, node_name, skip_cache=False):
        node_api = rosnode.get_api_uri(rospy.get_master(), node_name, skip_cache=skip_cache) #we need the PID but rosnode only provide XMLRPC URI
        try:
            code, msg, pid = ServerProxy(node_api[2]).getPid(ID) #get PID via XMLRPC URI
            if node_name in self.nodes:
                return self.nodes[node_name]
            else:
                try:
                    p = psutil.Process(pid)
                    self.nodes[node_name] = p
                    return p
                except:
                    return False
        except (SocketError, IOError) as e:
            if not skip_cache:
                return self.get_node_info(node_name, skip_cache=True)
            else:
                return False
    
    def get_process_stats(self):
        node_infos = self.get_all_node_info()
        all_node_stats = dict()
        for name, process in node_infos:
            with process.oneshot():
                node_stats=dict()
                psutil.cpu_count()
                single_cpus = process.cpu_percent(interval=None)
                node_stats['cpu_stats'] = {'cpu_percent_accumulated': single_cpus, 'cpu_percent': single_cpus/psutil.cpu_count()}
                node_stats['mem_stats'] = process.memory_info()._asdict()
                node_stats['io_stats'] = process.io_counters()._asdict()
                all_node_stats[name] = node_stats
        return all_node_stats

    def get_average_process_stats(self):
        node_infos = self.get_all_node_info()
        node_stats = {'cpu_usage_percent': 0.0, 'memory_usage': 0.0, 'io_writes': 0.0}
        for name, process in node_infos:
            with process.oneshot():
                cpu_usage = process.cpu_percent(interval=None)/psutil.cpu_count()
                memory_usage = process.memory_info().rss
                node_stats['cpu_usage_percent'] +=  cpu_usage
                node_stats['memory_usage'] += memory_usage
                node_stats['io_writes'] += process.io_counters().write_bytes
                if name == SIMULATOR_NODE_NAME:
                    node_stats[SIMULATOR_NODE_NAME + '_cpu_usage'] = cpu_usage # simulator node should only exist once
                    node_stats[SIMULATOR_NODE_NAME + '_memory_usage'] = memory_usage
        return node_stats
    
    def log_average_process_stats(self, freq, record_option):
        all_system_info = dict()
        is_on_process_level = record_option == RecordOption.ALL or record_option == RecordOption.NODE_STATS_ONLY
        is_on_overall_level = record_option == RecordOption.ALL or record_option == RecordOption.OVERALL_ONLY
        if is_on_process_level:
            all_system_info['ros_node_stats'] = {"cpu_usage_percent":0.0, "memory_usage": 0.0, "io_writes": 0.0, SIMULATOR_NODE_NAME + '_cpu_usage': 0.0, SIMULATOR_NODE_NAME + '_memory_usage': 0.0}
            io_writes_baseline = self.io_writes_baseline
        if is_on_overall_level:
            all_system_info['overall_stats'] = {"cpu_usage_percent":0.0, "memory_usage": 0.0, "io_writes": 0.0}
        
        counter = 0
        self.first_time_call()
        time.sleep(freq)
        while True:
            counter += 1
            if is_on_overall_level:
                overall_stats = self.overall_stats()
                cpu_usage, mem_usage, io_writes = overall_stats['cpu_stats']['cpu_percent'],overall_stats['mem_stats']['used'], overall_stats['io_stats']['write_bytes']
                all_system_info['overall_stats']['cpu_usage_percent'] += cpu_usage
                all_system_info['overall_stats']['memory_usage'] += mem_usage
                all_system_info['overall_stats']['io_writes'] = io_writes - io_writes_baseline # cumulated values
            
            if is_on_process_level:
                proc_stats = self.get_average_process_stats()
                cpu_usage, mem_usage, io_writes = proc_stats['cpu_usage_percent'],proc_stats['memory_usage'],proc_stats['io_writes']
                all_system_info['ros_node_stats']['cpu_usage_percent'] += cpu_usage
                all_system_info['ros_node_stats']['memory_usage'] += mem_usage
                all_system_info['ros_node_stats']['io_writes'] = io_writes if io_writes > all_system_info['ros_node_stats']['io_writes'] else all_system_info['ros_node_stats']['io_writes'] # Values are cummulative, but since we are operating on sum(byte_write) for all ROS processes, the value might decrease (if another node dies before this node writes its result). For average values we buy that behaviour, here it might result in completely wrong values (in the worst case)
                if proc_stats.__contains__(SIMULATOR_NODE_NAME + '_cpu_usage'):
                    all_system_info['ros_node_stats'][SIMULATOR_NODE_NAME + '_cpu_usage'] += proc_stats[SIMULATOR_NODE_NAME + '_cpu_usage']
                    all_system_info['ros_node_stats'][SIMULATOR_NODE_NAME + '_memory_usage'] += proc_stats[SIMULATOR_NODE_NAME + '_memory_usage']
        
            if self.termination_handler.kill_now == True: # the roslaunch should trigger the SIGKILL after the (ROS) monitoring node decides to shut down
                break
            time.sleep(freq)

        if is_on_process_level:
            all_system_info['ros_node_stats']['cpu_usage_percent'], all_system_info['ros_node_stats']['memory_usage'],all_system_info['ros_node_stats'][SIMULATOR_NODE_NAME + '_cpu_usage'],all_system_info['ros_node_stats'][SIMULATOR_NODE_NAME + '_memory_usage'] = map(lambda x: x/counter, [all_system_info['ros_node_stats']['cpu_usage_percent'],all_system_info['ros_node_stats']['memory_usage'],all_system_info['ros_node_stats'][SIMULATOR_NODE_NAME + '_cpu_usage'],all_system_info['ros_node_stats'][SIMULATOR_NODE_NAME + '_memory_usage']])
        if is_on_process_level:
            all_system_info['overall_stats']['cpu_usage_percent'], all_system_info['overall_stats']['memory_usage'] = map(lambda x: x/counter, [all_system_info['overall_stats']['cpu_usage_percent'],all_system_info['overall_stats']['memory_usage']])
        
        all_system_info['total_system_memory'] =  psutil.virtual_memory().total
        self.add_exec_duration(all_system_info)
        return all_system_info
                  

    def log_stats_ticked(self, freq, record_option):
        all_system_info = list()
        cur_time = 0.0
        self.first_time_call()
        time.sleep(freq)
        while True:
            stats=dict()
            if record_option == RecordOption.ALL or record_option == RecordOption.NODE_STATS_ONLY:
                stats['ros_node_stats'] = self.get_process_stats()
            if record_option == RecordOption.ALL or record_option == RecordOption.OVERALL_ONLY:
                stats['overall_stats'] = self.overall_stats()
            all_system_info.append({'time': cur_time,'stats': stats})
            cur_time += freq
            if self.termination_handler.kill_now == True: # the roslaunch should trigger the SIGKILL after the (ROS) monitoring node decides to shut down
                break
            time.sleep(freq)
        for d in all_system_info:
            self.add_exec_duration(d)
        return all_system_info

    def add_exec_duration(self, info_dict):
        info_dict['execution_duration'] = round(time.time()) - self.start_time


    
    def log_stats(self, freq, granularity, record_option):
        if granularity == Granularity.TICKED:
            logger.info("Logging performance data for every tick with a frequency of %.2f seconds", freq)
            all_system_info = self.log_stats_ticked(freq,record_option)
        if granularity == Granularity.AVERAGED:
            logger.info("Logging averaged performance data")
            all_system_info = self.log_average_process_stats(freq, record_option)
        
        self.write_to_file(all_system_info)

    def write_to_file(self, all_system_info):
        OUT_SUFFIX = datetime.now().strftime('%Y-%m-%d-%H-%M')
        file_name = OUT_FILE_NAME + OUT_SUFFIX + ".json" # use jsonlite (NOT rjosn!) with flatten option to import as df in R
        out_dest = os.path.join(BASE_DIR, file_name)
        logger.info("Writing data to file. Objects with performance data have as size of (in bytes): %d", sys.getsizeof(all_system_info))
        if not os.path.exists(os.path.dirname(out_dest)):
            try:
                os.makedirs(os.path.dirname(out_dest))
            except OSError as exc:
                if exc.errno != errno.EEXIST:
                    raise
        with open(out_dest,'w') as f_out:
            f_out.write(json.dumps(all_system_info,sort_keys=True))
        logger.info("Data written to: %s", out_dest)

    def terminate_execution(self):
        self.termination_handler.set_termination_flag(None,None)

    def initLogging(self):
        loggername = "PerformanceObserver:" + __name__
        global logger
        logger = logging.getLogger(loggername)
        logger.setLevel(logging.INFO)
        fh = logging.FileHandler(loggername + ".log")
        stdHandler = logging.StreamHandler(sys.stdout)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        stdHandler.setFormatter(formatter)
        fh.setFormatter(formatter)
        logger.addHandler(fh)
        logger.addHandler(stdHandler)

    def start(self):
        self.initLogging()
        system_observer = SystemObserver()
        system_observer.log_stats(0.5, Granularity.AVERAGED ,RecordOption.ALL)

if __name__ == "__main__":
    SystemObserver().start()
