#include <Servo.h>
#include <math.h>
#define _CMD_SET_X 1
#define _CMD_SET_Y 2
#define _CMD_SET_Z 3
#define _CMD_SET_INTERVAL 4
#define _CMD_GET_QUEUE_MAX 5

#define _QUEUE_MAX 75

#define base_pin 9
#define shoulder_pin 6
#define elbow_pin 5
#define gripper_pin 3

#define debug_mode true

#define _LINK_SERVO_MIN 550.0
#define _LINK_SERVO_MAX 2450.0
#define _LINK_SERVO_RANGE 1900.0

#define d1 175.0
#define d2 175.0

//Need to pre-compute this stuff if link length changes
#define d1_squared 30625.0
#define d2_squared 30625.0
#define d1_d2_doubled 61250.0

typedef struct Vector3{
    double x_or_base = 0.0;
    double y_or_shoulder = 0.0;
    double z_or_elbow = 0.0;
} Vector3;



const int _CFG_CMD_VAL_PLACES = 4; //we need 4 places for our payload (1000 to 2000)

Servo base, shoulder, elbow, gripper;
String cmd_string;
bool moving = false;
int cmd, cmd_id, cmd_val, cmd_key, i;
int interval = 50; // in milliseconds

Vector3 init_pos, final_pos;
Vector3 int_pos_array[_QUEUE_MAX];
int array_cursor = 0;

void debug_print(String msg){
    if(debug_mode){Serial.println(msg);}
}

int cartesian_to_us(Vector3 cartesian, Vector3 * us){
    double r, base, shoulder, elbow;

    if(cartesian.x_or_base == 0){return 0;} // TODO: deal with this better

    r = sqrt(pow(cartesian.x_or_base,2) + pow(cartesian.y_or_shoulder,2) + pow(cartesian.z_or_elbow,2));
    base = M_PI_2 - (atan(cartesian.y_or_shoulder/cartesian.x_or_base));
    elbow = M_PI - acos(-1 * (pow(r,2) - d1_squared - d2_squared) / (d1_d2_doubled));
    shoulder = M_PI_2 - asin(cartesian.z_or_elbow/r) - atan(d2*sin(elbow) / (d1+d2*cos(elbow))) + M_PI_2;

    base = base / M_PI * _LINK_SERVO_RANGE + _LINK_SERVO_MIN;
    elbow = elbow / M_PI * _LINK_SERVO_RANGE + _LINK_SERVO_MIN;
    shoulder = shoulder / M_PI * _LINK_SERVO_RANGE + _LINK_SERVO_MIN;

    (*us).x_or_base = base;
    (*us).y_or_shoulder = shoulder;
    (*us).z_or_elbow = elbow;

    return 1;
}

int find_intermediate_positions(Vector3 init_pos, Vector3 final_pos, Vector3 * int_pos_array, int array_len){
    Vector3 curr_pos, curr_us, increment;
    int idx = 0;
    float array_len_float = float(array_len);

    //set curr_pos to init_pos for now
    curr_pos = init_pos;

    //calculate increment
    increment.x_or_base = (final_pos.x_or_base - init_pos.x_or_base) / array_len_float;
    increment.y_or_shoulder = (final_pos.y_or_shoulder - init_pos.y_or_shoulder) / array_len_float;
    increment.z_or_elbow = (final_pos.z_or_elbow - init_pos.z_or_elbow) / array_len_float;

    //calculate intermediate positions
    for(idx=0;idx<array_len;idx++){
        curr_pos.x_or_base += increment.x_or_base;
        curr_pos.y_or_shoulder += increment.y_or_shoulder;
        curr_pos.z_or_elbow += increment.z_or_elbow;
        cartesian_to_us(curr_pos, &curr_us);
        int_pos_array[idx] = curr_us;
    }

    return 1;
}

void setup() {
    // put your setup code here, to run once
    base.attach(base_pin);
    shoulder.attach(shoulder_pin);
    elbow.attach(elbow_pin);
    gripper.attach(gripper_pin);

    init_pos.x_or_base = 100.0;
    init_pos.y_or_shoulder = 100.0;
    init_pos.z_or_elbow = 100.0;

    cartesian_to_us(init_pos, &final_pos);

    base.writeMicroseconds(final_pos.x_or_base);
    shoulder.writeMicroseconds(final_pos.y_or_shoulder);
    elbow.writeMicroseconds(final_pos.z_or_elbow);

    final_pos.x_or_base = 0.0;
    final_pos.y_or_shoulder = 0.0;
    final_pos.z_or_elbow = 0.0;
    
    Serial.begin(9600);
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
        cmd_id = cmd_string.toInt();
        debug_print("Received cmd_id:");
        debug_print(String(cmd));
        cmd_string = Serial.readStringUntil('/');
        cmd_val = cmd_string.toInt();

        switch(cmd_id){
            case _CMD_SET_X:
                if(cmd_val >= 10 && cmd_val <= 200){
                    final_pos.x_or_base = cmd_val;
                    debug_print("X set");
                }
                break;
            case _CMD_SET_Y:
                if(cmd_val >= -150 && cmd_val <= 150){
                    final_pos.y_or_shoulder = cmd_val;
                    debug_print("Y set");
                }
                break;
            case _CMD_SET_Z:
                if(cmd_val >= -50 && cmd_val <= 200){
                    final_pos.z_or_elbow = cmd_val;
                    debug_print("Z set");
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

        if (final_pos.x_or_base && final_pos.z_or_elbow && final_pos.y_or_shoulder){
            find_intermediate_positions(init_pos, final_pos, int_pos_array, _QUEUE_MAX);
            init_pos = final_pos;
            final_pos.x_or_base = 0.0;
            final_pos.y_or_shoulder = 0.0;
            final_pos.z_or_elbow = 0.0;
            moving = true;
            }
        }

    // if moving
    if(moving){
        debug_print("Begin moving");
        base.writeMicroseconds(int_pos_array[array_cursor].x_or_base);
        shoulder.writeMicroseconds(int_pos_array[array_cursor].y_or_shoulder);
        elbow.writeMicroseconds(int_pos_array[array_cursor].z_or_elbow);
        debug_print("Wrote to all servos");
        array_cursor++; 
        debug_print("Increased cursor");
        delay(interval);
        if(array_cursor >= _QUEUE_MAX){
            debug_print("Done moving");
            moving = false;
            array_cursor = 0;
        }
    }
}