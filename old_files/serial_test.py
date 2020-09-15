import serial, struct, time

CMD_SET_BASE = 1
CMD_SET_SHOULDER = 2
CMD_SET_ELBOW = 3
CMD_SET_INTERVAL = 4
CMD_GET_QUEUE_MAX = 5
CFG_CMD_VAL_PLACES = 4

def send_cmd(ser, cmd_id, cmd_val):
    cmd_id_shift = 10**CFG_CMD_VAL_PLACES 
    if cmd_id_shift < cmd_val:
        print("Invalid command value received")
        return None
    cmd = cmd_id_shift * cmd_id + cmd_val
    print("[IK_Node] sending cmd: %d"%cmd)
    if ser == None:
        return None
    ser.write(struct.pack('@l',cmd))

ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1.0)
send_cmd(ser, CMD_SET_BASE, 1000)
send_cmd(ser, CMD_SET_SHOULDER, 1400)
send_cmd(ser, CMD_SET_ELBOW, 1500)

while True:
    time.sleep(10)
    print("Waiting...")


