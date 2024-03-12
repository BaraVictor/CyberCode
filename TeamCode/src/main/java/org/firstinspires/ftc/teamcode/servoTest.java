package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "servoTest", group = "B")
public class servoTest extends OpMode{
    private Servo servo;
    double open1=0.71;
    double closed1=0.89;
    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger > 0){
            servo.setPosition(closed1);
        }
        if(gamepad1.right_bumper){
            servo.setPosition(open1);
        }
    }
}
