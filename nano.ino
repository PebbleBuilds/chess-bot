#include <Servo.h>
#define _CMD_SET_BASE 1
#define _CMD_SET_SHOULDER 2
#define _CMD_SET_ELBOW 3
#define _CMD_SET_INTERVAL 4
#define _CMD_CHECK_MOVING 5

#define _QUEUE_MAX 50

#define base_pin 3
#define shoulder_pin 5
#define elbow_pin 6
#define gripper_pin 9

const int _CFG_CMD_VAL_PLACES = 4; //we need 4 places for our payload (1000 to 2000)



typedef struct angle_set{
    int base_us = 0;
    int shoulder_us = 0;
    int elbow_us = 0;
} angle_set;

Servo base, shoulder, elbow, gripper;
bool moving = false;
int cmd, cmd_id, cmd_val;
int interval = 100; // in milliseconds

angle_set queue[_QUEUE_MAX];
int queue_cursor = 0;
int cmd_key, i;

void setup() {
    // put your setup code here, to run once
    base.attach(base_pin);
    shoulder.attach(shoulder_pin);
    elbow.attach(elbow_pin);
    gripper.attach(gripper_pin);

    base.writeMicroseconds(1500);
    shoulder.writeMicroseconds(1500);
    elbow.writeMicroseconds(1500);
    
    Serial.begin(9600);
    cmd_key = 1;
    for(i;_CFG_CMD_VAL_PLACES;i++){
      cmd_key = cmd_key * 10;
    }
}

void loop() {

    // if waiting for command
    if((!moving) && Serial.available()){
        cmd = Serial.read(); // cmd will be stored in an int.
        cmd_id = cmd / cmd_key;
        cmd_val = cmd % cmd_key;

        switch(cmd_id){
            case _CMD_SET_BASE:
                if(cmd_val >= 1000 && cmd_val <= 2000){
                    queue[queue_cursor].base_us = cmd_val;
                }
                break;
            case _CMD_SET_SHOULDER:
                if(cmd_val >= 1000 && cmd_val <= 2000){
                    queue[queue_cursor].shoulder_us = cmd_val;
                }
                break;
            case _CMD_SET_ELBOW:
                if(cmd_val >= 1000 && cmd_val <= 2000){
                    queue[queue_cursor].elbow_us = cmd_val;
                }
                break;
            case _CMD_SET_INTERVAL:
                if(cmd_val > 0){
                    interval = cmd_val;
                }
                break;
            case _CMD_CHECK_MOVING:
                if(moving){
                    Serial.write(1);
                }
                else{
                    Serial.write(0);
                }
                break;
        }

        if (queue[queue_cursor].base_us && queue[queue_cursor].elbow_us && queue[queue_cursor].shoulder_us){
            queue_cursor++;
            if(queue_cursor >= _QUEUE_MAX){
                moving = true;
                queue_cursor = 0;
            }
        }
    }

    // if moving
    if(moving){
        if(millis() % interval == 0){
            base.write(queue[queue_cursor].base_us);
            shoulder.write(queue[queue_cursor].shoulder_us);
            elbow.write(queue[queue_cursor].elbow_us);
            queue[queue_cursor].base_us = 0;
            queue[queue_cursor].shoulder_us = 0;
            queue[queue_cursor].elbow_us = 0;
            queue_cursor++; 
        }
        if(queue_cursor >= _QUEUE_MAX){
            moving = false;
            queue_cursor = 0;
            Serial.write("done");
        }
    }
}