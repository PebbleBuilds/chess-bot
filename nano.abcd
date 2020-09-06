#include <Servo.h>
#define _CMD_SET_BASE 1
#define _CMD_SET_SHOULDER 2
#define _CMD_SET_ELBOW 3
#define _CMD_SET_INTERVAL 4
#define _CMD_GET_QUEUE_MAX 5

#define _QUEUE_MAX 5

#define base_pin 9
#define shoulder_pin 6
#define elbow_pin 5
#define gripper_pin 3

#define debug_mode false

#define _LINK_SERVO_MIN 550
#define _LINK_SERVO_MAX 2450

void debug_print(String msg){
    if(debug_mode){Serial.println(msg);}
}

const int _CFG_CMD_VAL_PLACES = 4; //we need 4 places for our payload (1000 to 2000)

typedef struct angle_set{
    int base_us = 0;
    int shoulder_us = 0;
    int elbow_us = 0;
} angle_set;

Servo base, shoulder, elbow, gripper;
String cmd_string;
bool moving = false;
int cmd, cmd_id, cmd_val, cmd_key, i;
int interval = 10; // in milliseconds

angle_set queue[_QUEUE_MAX];
int queue_cursor = 0;

void setup() {
    // put your setup code here, to run once
    base.attach(base_pin);
    shoulder.attach(shoulder_pin);
    elbow.attach(elbow_pin);
    gripper.attach(gripper_pin);

    base.writeMicroseconds(1500);
    shoulder.writeMicroseconds(1400);
    elbow.writeMicroseconds(1500);
    
    Serial.begin(2400);
    cmd_key = 1;
    for(i=0;i<_CFG_CMD_VAL_PLACES;i++){
      cmd_key = cmd_key * 10;
    }
    debug_print("Setup done");
}

void loop() {
    // if waiting for command
    if((!moving) && Serial.available()){
        cmd_string = Serial.readStringUntil('/');
        cmd = cmd_string.toInt();
        debug_print("Received:");
        debug_print(String(cmd));
        cmd_id = cmd / cmd_key;
        cmd_val = cmd % cmd_key;

        switch(cmd_id){
            case _CMD_SET_BASE:
                if(cmd_val >= _LINK_SERVO_MIN && cmd_val <= _LINK_SERVO_MAX){
                    queue[queue_cursor].base_us = cmd_val;
                    debug_print("Base set");
                }
                break;
            case _CMD_SET_SHOULDER:
                if(cmd_val >= _LINK_SERVO_MIN && cmd_val <= _LINK_SERVO_MAX){
                    queue[queue_cursor].shoulder_us = cmd_val;
                    debug_print("Shoulder set");
                }
                break;
            case _CMD_SET_ELBOW:
                if(cmd_val >= _LINK_SERVO_MIN && cmd_val <= _LINK_SERVO_MAX){
                    queue[queue_cursor].elbow_us = cmd_val;
                    debug_print("Elbow set");
                }
                break;
            case _CMD_SET_INTERVAL:
                if(cmd_val > 0){
                    interval = cmd_val;
                    debug_print("Interval Set");
                }
                break;
            case _CMD_GET_QUEUE_MAX:
                Serial.write(_QUEUE_MAX);
                break;
        }

        if (queue[queue_cursor].base_us && queue[queue_cursor].elbow_us && queue[queue_cursor].shoulder_us){
            queue_cursor++;
            debug_print("Increasing cursor");
            if(queue_cursor >= _QUEUE_MAX){
                moving = true;
                queue_cursor = 0;
                debug_print("Switching to moving");
            }
        }
    }

    // if moving
    if(moving){
        debug_print("Begin moving");
        base.writeMicroseconds(queue[queue_cursor].base_us);
        shoulder.writeMicroseconds(queue[queue_cursor].shoulder_us);
        elbow.writeMicroseconds(queue[queue_cursor].elbow_us);
        debug_print("Wrote to all servos");
        queue[queue_cursor].base_us = 0;
        queue[queue_cursor].shoulder_us = 0;
        queue[queue_cursor].elbow_us = 0;
        debug_print("Emptied queue slots");
        queue_cursor++; 
        debug_print("Increased cursor");
        delay(interval);
        if(queue_cursor >= _QUEUE_MAX){
            debug_print("Done moving");
            moving = false;
            queue_cursor = 0;
        }
    }
}