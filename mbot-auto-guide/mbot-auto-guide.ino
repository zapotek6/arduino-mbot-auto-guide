#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#include <MeOrion.h>

double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;
void AnalyzeContext(double distance);
double context;
double _freeSpace;
double _collision;
double _approacing;
void EvaluateReaction(double context);
double keepMotorOnTime;
double action;
double _forward;
double speed;
double speedFactor;
double distance;
double _reverse;
double _stop;
void ExecuteAction(double action, double speed);
double motorSpeed1;
double motorSpeed2;
double _turnLeft;
double _turnRight;
MeDCMotor motor_9(M1);
MeDCMotor motor_10(M2);
void RCDecoder(double buttonCode);
double _RC_A;
double _RC_B;
double _RC_C;
double _RC_D;
double _RC_E;
double _RC_UP;
double _RC_LEFT;
double _RC_RIGHT;
double _RC_DOWN;
double _RC_SETTINGS;
double run;
double _max_speedFactor;
double _min_speedFactor;
double mode;
double _mode_AUTO;
double _mode_MANUAL;
double _default_speedFactor;
double irButton;
void ChooseLeftOrRight();
double lastRun;
MeUltrasonicSensor ultrasonic_3(3);
MeInfraredReceiver ir_6(6);


void AnalyzeContext(double distance)
{
    if((distance) > (30)){
        context = _freeSpace;
    }else{
        if((distance) < (10)){
            context = _collision;
        }else{
            context = _approacing;
        }
    }
    
}

void EvaluateReaction(double context)
{
    if(((context)==(_freeSpace))){
        keepMotorOnTime = 0;
        action = _forward;
        speed = ((3) * ((speedFactor) / (5))) * (distance);
        if((speed) > (255)){
            speed = 255;
        }else{
            speed = ((3) * ((speedFactor) / (5))) * (distance);
        }
    }else{
        if(((context)==(_approacing))){
            ChooseLeftOrRight();
        }else{
            if(((context)==(_collision))){
                action = _reverse;
                speed = 100;
                keepMotorOnTime = 2;
            }else{
                action = _stop;
                speed = 0;
            }
        }
    }
    
}

void ExecuteAction(double action, double speed)
{
    if(((action)==(_forward))){
        motorSpeed1 = speed;
        motorSpeed2 = speed;
    }
    
    if(((action)==(_reverse))){
        motorSpeed1 = (speed) * (-1);
        motorSpeed2 = (speed) * (-1);
    }
    
    if(((action)==(_turnLeft))){
        motorSpeed1 = speed;
        motorSpeed2 = (speed) * (-1);
    }
    
    if(((action)==(_turnRight))){
        motorSpeed1 = (speed) * (-1);
        motorSpeed2 = speed;
    }
    
    if(((action)==(_stop))){
        motorSpeed1 = 0;
        motorSpeed2 = 0;
        keepMotorOnTime = 0;
    }
    
    motor_9.run(motorSpeed1);
    
    motor_10.run(motorSpeed2);
    
    _delay(keepMotorOnTime);
    
}

void RCDecoder(double buttonCode)
{
    _RC_A = 69;
    
    _RC_B = 70;
    
    _RC_C = 71;
    
    _RC_D = 68;
    
    _RC_E = 67;
    
    _RC_UP = 64;
    
    _RC_LEFT = 9;
    
    _RC_RIGHT = 7;
    
    _RC_DOWN = 25;
    
    _RC_SETTINGS = 21;
    
    if(((buttonCode)==(_RC_B))){
        run = 0;
        action = _stop;
    }
    
    if(((buttonCode)==(_RC_A))){
        run = 1;
    }
    
    if(((buttonCode)==(_RC_DOWN))){
        action = _reverse;
    }
    
    if(((buttonCode)==(_RC_UP))){
        action = _forward;
    }
    
    if(((buttonCode)==(_RC_RIGHT))){
        action = _turnRight;
        keepMotorOnTime = 0;
    }
    
    if(((buttonCode)==(_RC_LEFT))){
        action = _turnLeft;
        keepMotorOnTime = 0;
    }
    
    if(((buttonCode)==(_RC_C))){
        if((speedFactor) < (_max_speedFactor)){
            speedFactor = (speedFactor) + (1);
        }
    }
    
    if(((buttonCode)==(_RC_E))){
        if((speedFactor) > (_min_speedFactor)){
            speedFactor = (speedFactor) - (1);
        }
    }
    
    if(((buttonCode)==(_RC_D))){
        if(((mode)==(_mode_AUTO))){
            mode = _mode_MANUAL;
        }else{
            mode = _mode_AUTO;
        }
    }
    
    if(((buttonCode)==(_RC_SETTINGS))){
        run = 0;
        action = _stop;
        mode = _mode_AUTO;
        speedFactor = _default_speedFactor;
    }
    
    irButton = 0;
    
}

void ChooseLeftOrRight()
{
    if((random(1,(10)+1)) > (5)){
        action = _turnLeft;
        speed = 150;
        keepMotorOnTime = 1;
    }else{
        action = _turnRight;
        speed = 150;
        keepMotorOnTime = 1;
    }
    
}


void setup(){
    ir_6.begin();
    _freeSpace = 0;
    _collision = 1;
    _approacing = 2;
    _forward = 0;
    _reverse = 1;
    _turnLeft = 2;
    _turnRight = 3;
    _stop = 4;
    motorSpeed1 = 0;
    motorSpeed2 = 0;
    keepMotorOnTime = 0;
    speed = 0;
    action = _stop;
    _default_speedFactor = 3;
    _min_speedFactor = 1;
    _max_speedFactor = 5;
    speedFactor = _default_speedFactor;
    run = 0;
    lastRun = 0;
    _mode_AUTO = 0;
    _mode_MANUAL = 1;
    mode = 0;
    
}

void loop(){
    
    distance = ultrasonic_3.distanceCm();
    irButton = ir_6.getCode();
    if(((mode)==(_mode_AUTO))){
        if((((run)==(1))) || (((lastRun)==(1)))){
            AnalyzeContext(distance);
            EvaluateReaction(context);
            RCDecoder(irButton);
        }
        lastRun = 0;
    }else{
        speed = 150;
        RCDecoder(irButton);
    }
    ExecuteAction(action,speed);
    action = _stop;
    RCDecoder(irButton);
    
    _loop();
}

void _delay(float seconds){
    long endTime = millis() + seconds * 1000;
    while(millis() < endTime)_loop();
}

void _loop(){
    ir_6.loop();
    
    
}
