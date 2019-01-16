from odrive_ros import odrive_interface
import time

od = odrive_interface.ODriveInterfaceAPI(1)
od.connect()
od.calibrate()
od.engage()

od.drive(0,1000)

time.sleep(5)

od.drive(0,0)
od.release()

print("finished")
