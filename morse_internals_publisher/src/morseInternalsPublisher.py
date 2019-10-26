#!/usr/bin/env python3


import pymorse
import rospy
from std_msgs.msg import String
import std_msgs.msg
import functools
import sys
import threading
from multiprocessing.dummy import Pool as ThreadPool 

_simulation_rpcs = None
_communication_rpcs = None
_time_rpcs = None
_frequency = None



def read_parameters():
    global _simulation_rpcs
    _simulation_rpcs = eval(rospy.get_param(rospy.get_name() + "/simulation_services")) # We have to trust our parameters, parsing YAML would be better
    global _communication_rpcs
    _communication_rpcs = eval(rospy.get_param(rospy.get_name() + "/communication_services"))
    global _time_rpcs
    _time_rpcs = eval(rospy.get_param(rospy.get_name() + "/time_services"))
    global _frequency
    _frequency = rospy.get_param(rospy.get_name() + '/frequency', None)



def create_publishers(rpc_type, rpc_commands):
    rpc_publishers = []
    for rpc in rpc_commands:
        pub = rospy.Publisher("/morse_internals/" + rpc[0] , String, queue_size=1)
        pub.morseCall = functools.partial(pymorse.Morse().rpc, rpc_type, *rpc)
        rpc_publishers.append(pub)
    return(rpc_publishers)

def init_morse_publishers(simulator):
    allMorsePublishers = []

    # Morse provides three kinds of services: supervision services ('simulation'),
    # communication services ('communication') and time services ('time')
    # see: https://www.openrobots.org/morse/doc/latest/user/code/morse.services.html#module-morse.services.communication_services
    
    if _simulation_rpcs is not None:
        allMorsePublishers.extend(create_publishers('simulation', _simulation_rpcs))
    if _communication_rpcs is not None:
        allMorsePublishers.extend(create_publishers('communication', _communication_rpcs))
    if _communication_rpcs is not None:
        allMorsePublishers.extend(create_publishers('time', _time_rpcs))

    #Add new calls here
    #pub = ...
    #pub.morseCall = ...
    #allMorsePublishers.append(pub)
    return(allMorsePublishers)

def publish_topic(publisher):
    result = str(publisher.morseCall()) #we have to explicitly convert it
    result.replace("\n", " ") # Strip line breaks from string, otherwise assertion error will occur :S!
    publisher.publish(result)



def publishMessages(publishers, frequency):
    """
    Start publishing of MORSE internals
    @param publishers: set of publishers
    @type  publishers list
    @param frequency: publication frequency in hz
    @type  publishers int
    """
    print(frequency)
    pool = ThreadPool(len(publishers))
    rate = rospy.Rate(frequency)
    while not rospy.core.is_shutdown():
        pool.map(publish_topic,publishers)
        #for pub in publishers:
            #threading.Thread(target = publish_topic, args = (pub,)).start()
        rate.sleep()

def startMorsePublisher(nodeName="morse_internals", pubFrequency=1):
    """
    First inits a node withe the specified name, then creates publishers and
    starts publishing the information.
    Careful: Publishers interface with MORSE via sockets and then publish the
    information under the respective topic. The frequency directly impacts the
    number of mesages sent via sockets and might have an impact on performance.
    @param nodeName: name of node
    @type  nodeName str
    @param pubFrequency: publication frequency in hz
    @type  pubFrequency int
    """
    with pymorse.Morse() as morseSim: #To make sure that the sockets are closed after ros is shutdown
        rospy.init_node(nodeName)
        read_parameters()
        if _frequency is not None:
            pubFrequency = _frequency
        allMorsePublishers = init_morse_publishers(morseSim)
        publishMessages(allMorsePublishers, pubFrequency)

if __name__ == "__main__":
    if sys.version_info[0] < 3: #If we run it manually, when running from launch file shebang handles this
        rospy.logfatal("Must be using Python 3. Pymorse requires Python 3.") 
        raise Exception("Must be using Python 3. Pymorse requires Python 3.")
    startMorsePublisher()
