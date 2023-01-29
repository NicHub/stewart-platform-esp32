"""

S T E W A R T    P L A T F O R M    O N    E S P 3 2

run with:
python gcode2serial.py -p /dev/cu.SLAB_USBtoUART -f demo_1.gcode

"""

import time
import serial
import time
import argparse

wait_for_ans = False

parser = argparse.ArgumentParser(description="HEXAPOD GCODE SENDER")
parser.add_argument("-p", "--port", help="Input USB port", required=True)
parser.add_argument("-f", "--file", help="Gcode file name", required=True)
args = parser.parse_args()

# Display input values
print("USB Port: %s" % args.port)
print("Gcode file: %s" % args.file)


def removeComment(string):
    if string.find(";") == -1:
        return string
    else:
        return string[: string.index(";")]


# Open serial port
hx_serial = serial.Serial(args.port, 115200)
print("Opening Serial Port")

# Wake up
# Hit enter a few times to wake the hexapod
hx_serial.write(str.encode("\r\n\r\n"))
time.sleep(1)
hx_serial.flushInput()
print("SENDING GCODE")

endlessLoop = False

while True:
    # Open g-code file
    f = open(args.file, "r")
    print("Opening gcode file")

    # Stream g-code
    for line in f:
        cmd = removeComment(line).strip()

        if cmd.isspace() == False and len(cmd) > 0:
            print(cmd)
            if cmd.find("G4 P") >= 0:
                t = float(cmd[5:])
                print(f"Waiting: {t}")
            else:
                t = 0.002
            time.sleep(t)
            hx_serial.write(str.encode(cmd + "\n"))
            if wait_for_ans:
                while True:
                    try:
                        hx_ans = hx_serial.readline().strip().decode("ascii")
                        print(":" + hx_ans)
                    except:
                        pass
    if not endlessLoop:
        break


# Close file and serial port
f.close()

hx_serial.close()
