import anki_vector
import time
import numpy as np

if __name__ == '__main__':
    # Serial Numbers of robots to sleep. Probably should be an argument but maybe later
    serialList = ["007039c3","00603393","00702c90","00702e03","005031a6","0040161d","00903ba4"]
    # serialList = ["00903ba4"]
    realRobotList = []
    for serialNum in serialList:
        realRobotList.append(anki_vector.Robot(serialNum))
        realRobotList[-1].connect()
    for serial in serialList[:-1]:
        try:
            #Try to put the robot to sleep
            anki_vector.behavior.ReserveBehaviorControl(serial=serial)._conn.connect()
            with anki_vector.Robot(serial=serial, default_logging=False) as robot:
                # Lock the robots head in place facing up to see color
                robot.behavior.set_head_angle(anki_vector.util.degrees(45))
            print('Successfully stopped ' + serial)
        except:
            print('Could not find ' + serial)

    for i in range(np.size(realRobotList)):
        batt = realRobotList[i].get_battery_state().battery_volts
        print('robot {}: {}'.format(i,batt))
    while True:
        print("Checking battery every 2 minutes")
        for i in range(np.size(realRobotList)):
            batt = realRobotList[i].get_battery_state().battery_volts
            if float(batt) < 3.7:
                print('low battery for robot {}'.format(i))
        time.sleep(120)
