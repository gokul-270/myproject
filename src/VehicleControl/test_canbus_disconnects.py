import can
import threading

bus = can.interface.Bus(bustype='socketcan', channel='can0')

# Keep program running indefinitely
def check_can_connection(bus):
    while True:
        if bus.state != can.BusState.ACTIVE:
            print("CANbus connection lost! Program terminated.")
            # Insert code here to kill program
            raise SystemExit
        else:
            # Wait for a short time before checking connection again
            threading.Event().wait(1)


check_can_thread = threading.Thread(target=check_can_connection)
check_can_thread.start()

while True:
    # Insert program logic here
    pass
