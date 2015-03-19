### sends a continuous pulse
from core import serial_command
import sys
import time

def safe_exit(sc,e):
    print "Exit safely"
    print e
    sc.stop()

if __name__=="__main__":
    width = sys.argv[1]
    channel = sys.argv[2]
    number = sys.argv[3]
    width = int(width)
    channel = int(channel)
    number = int(number)
    print "Opening serial link..."
    sc = serial_command.SerialCommand("/dev/tty.usbserial-FTE3C0PG")
    print "Done!"
    sc.stop()
    sc.select_channel(channel)
    sc.set_pulse_height(16383)
    sc.set_pulse_width(width)
    sc.set_pulse_number(number)
    for i in range(10):
        try:
            sc.trigger_averaged()
            pin = False
            while pin==False:
                pin = sc.poll_for_avg_pin(timeout=1.0)
                #print sc.stop_triggering(), pin
            print pin
        except Exception,e:
            safe_exit(sc,e)
        except KeyboardInterrupt:
            safe_exit(sc,"keyboard interrupt")
        
        
