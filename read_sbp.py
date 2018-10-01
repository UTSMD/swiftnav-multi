#!/usr/bin/env python
import rospy
#from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.navigation import SBP_MSG_BASELINE_NED, SBP_MSG_POS_LLH, \
    SBP_MSG_VEL_NED, SBP_MSG_GPS_TIME

import argparse

class RtkMessage:
    '''
    Saves and outputs parsed RTK data from Piks
    '''

    def __init__(self):
        self.flag = 0.0
        self.n = 0.0
        self.e = 0.0
        self.d = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.h = 0.0
        self.v_n = 0.0
        self.v_e = 0.0
        self.v_d = 0.0
        self.wn = 0
        self.tow = 0

    def whole_string(self):
        '''
        Returns all the data as a string
        '''

        return('%.0f\t%.0f\t%2.8f\t%2.8f\t%4.6f\t%6.0f\t%6.0f\t%6.0f\t'
               '%6.0f\t%6.0f\t%6.0f\t%.0f\t' %
               (self.wn, self.tow, self.lat, self.lon, self.h, self.n, self.e,
                self.d, self.v_n, self.v_e, self.v_d, self.flag))


def read_rtk(port='/dev/ttyUSB0', baud=230400):
    '''
    Reads the RTK output from SwiftNav Piksi, parses the messege and prints.
    Piksi's must be configured to give RTK message through the serial port.
    NOTE: Current official sbp drivers only support python-2

    Args:
        port: serial port [[default='/dev/ttyUSB0']
        baud: baud rate [default=230400]

    Returns:
        None
    '''
    
    #setupRosPublisher
    publishers = {}
    publishers['pose_fix'] = rospy.Publisher(rospy.get_name() + '/NavSatFix',
                                                 NavSatFix, queue_size=10)
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('piksi')#, anonymous=True)
    rospy.sleep(0.5)  # Wait for a while for init to complete before printing.
    rospy.loginfo(rospy.get_name() + " start")
    
    #rate = rospy.Rate(10) #10hz
    
    print('Reading from {} at {}'.format(port, baud))

    m = RtkMessage()
    # t_now = datetime.now().strftime('%Y%m%d%H%M%S')
    # out_file = 'GPS_' + t_now + '.txt'

    # open a connection to Piksi
    # with open(out_file, 'w') as f:
    with PySerialDriver(port, baud) as driver:
        with Handler(Framer(driver.read, None, verbose=True)) as source:
            try:
            #while not rospy.is_shutdown():
                msg_list = [SBP_MSG_BASELINE_NED, SBP_MSG_POS_LLH,
                            SBP_MSG_VEL_NED, SBP_MSG_GPS_TIME]
                for msg, metadata in source.filter(msg_list):

                    # LLH position in deg-deg-m
                    if msg.msg_type == 522:
                        m.lat = msg.lat
                        m.lon = msg.lon
                        m.h = msg.height

                    # RTK position in mm (from base to rover)
                    elif msg.msg_type == 524:
                        m.n = msg.n
                        m.e = msg.e
                        m.d = msg.d
                        m.flag = msg.flags

                    # RTK velocity in mm/s
                    elif msg.msg_type == 526:
                        m.v_n = msg.n
                        m.v_e = msg.e
                        m.v_d = msg.d

                    # GPS time
                    elif msg.msg_type == 258:
                        m.wn = msg.wn
                        m.tow = msg.tow  # in millis

                    else:
                        pass
                    
                    # Navsatfix message.
                    navsatfix_msg = NavSatFix()
                    navsatfix_msg.header.stamp = rospy.Time.now()
                    navsatfix_msg.header.frame_id = 'gps'
                    navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                    navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS
                    navsatfix_msg.latitude = m.lat
                    navsatfix_msg.longitude = m.lon
                    navsatfix_msg.altitude = m.h
                    navsatfix_msg.status.status = NavSatStatus.STATUS_FIX
                    navsatfix_msg.position_covariance = [1.0, 0, 0,
                                                         0, 1.0, 0,
                                                         0, 0, 1.0]
                    
                    publishers['pose_fix'].publish(navsatfix_msg)
                    #print(m.whole_string())
                    # f.write(line)
                    # f.write('\n')
                    
                    if rospy.is_shutdown():
                        break

            except KeyboardInterrupt:
                pass

    return


if __name__ == "__main__":
    try:
        parser = argparse.ArgumentParser(
            description=(
                'Opens and reads the output of SwiftNav Piksi. \
                Developed based on Swift Navigation SBP example.'))
        parser.add_argument(
            '-p', '--port',
            default=['/dev/ttyUSB0'],
            nargs=1,
            help='specify the serial port to use [default = \'/dev/ttyUSB0\']')
        parser.add_argument(
            '-b', '--baud',
            default=[230400],
            nargs=1,
            help='specify the baud rate [default = 230400]')
        args = parser.parse_args()

        read_rtk(args.port[0], args.baud[0])
    except rospy.ROSInterruptException:
        pass
