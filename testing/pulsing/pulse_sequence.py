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
    rate = sys.argv[2]
    channel = sys.argv[3]
    number = sys.argv[4]
    width = int(width)
    rate = float(rate)
    number = int(number)
    print width,rate
    sc = serial_command.SerialCommand("/dev/tty.usbserial-FTGA2OCZ")
    sc.stop()
    sc.select_channel(channel)
    sc.set_pulse_height(16383)
    sc.set_pulse_width(width)
    sc.set_pulse_delay(rate)
    sc.set_pulse_number(number)
    sc.set_trigger_delay(0)
    try:
        sc.fire_sequence()
    except Exception,e:
        safe_exit(sc,e)
    except KeyboardInterrupt:
        safe_exit(sc,"keyboard interrupt")
    print "Firing! Waiting for sequence to finish..."
    rate = 1/(rate*1e-3)
    time.sleep( (number / rate) + 1 )
    pin, rms, chan = sc.read_pin_sequence()
    print "PIN respoinse: %s +/- %s" % (pin[chan[0]], rms[chan[0]])
        
