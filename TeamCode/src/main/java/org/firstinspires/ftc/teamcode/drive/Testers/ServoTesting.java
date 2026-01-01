package org.firstinspires.ftc.teamcode.drive.Testers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//FTC Decode 2026 IntakeTesting
public class ServoTesting {
    Servo ServoTest;
    Gamepad gamepad_2;
    public ServoTesting(HardwareMap hwdmap, Gamepad gmpd){
        gamepad_2 = gmpd;
        ServoTest = hwdmap.get(Servo.class,"ServoTest");
    }

    double max_position = 1;
    double min_position = 0;

    public double current_position = min_position;
    public double last_position = 0;
    boolean triggerButton = false;
    boolean historyTrigger = false;
    double temp_last_position = 0;

    public void Run(){
        if(gamepad_2.left_bumper && current_position > min_position){
            if(!historyTrigger){
                last_position = temp_last_position;
                temp_last_position = current_position;
                historyTrigger = true;
            }
            if(!triggerButton){
                current_position = current_position - 0.01;
                triggerButton = true;
            }
        }else if(gamepad_2.right_bumper && current_position < max_position){
            if(!historyTrigger){
                last_position = temp_last_position;
                temp_last_position = current_position;
                historyTrigger = true;
            }
            if(!triggerButton){
                current_position = current_position + 0.01;
                triggerButton = true;
            }
        }else{
            triggerButton = false;
        }

        if(gamepad_2.a){
            ServoTest.setPosition(current_position);
            last_position = temp_last_position;
            historyTrigger = false;
        }else if(gamepad_2.y){
            last_position = temp_last_position;
            current_position = last_position;
            ServoTest.setPosition(last_position);
        }
    }
}
