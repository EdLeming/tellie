### sends a continuous pulse
from core import serial_command
import sys

def safe_exit(sc,e):
    print "Exit safely"
    print e
    sc.stop()

if __name__=="__main__":
    width = sys.argv[1]
    rate = sys.argv[2]
    channel = sys.argv[3]
    width = int(width)
    rate = float(rate)
    channel = int(channel)
    print width,rate
    sc = serial_command.SerialCommand("/dev/tty.usbserial-FTGA2OCZ")
    sc.stop()
    sc.select_channel(channel)
    sc.set_pulse_height(16383)
    sc.set_pulse_width(width)
    sc.set_pulse_delay(rate)
    try:
        sc.fire_continuous()
        while True:
            pass
    except Exception,e:
        safe_exit(sc,e)
    except KeyboardInterrupt:
        safe_exit(sc,"keyboard interrupt")
        
        
