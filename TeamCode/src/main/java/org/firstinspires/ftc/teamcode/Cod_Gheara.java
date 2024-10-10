package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "Cod_Gheara", group = "A")
public class Cod_Gheara extends OpMode {
    public Servo demoServoRight;
    public Servo demoServoLeft;
    double open1=0.71;
    double closed1=0.87;
    double open2=0.75;
    double closed2=0.56;
    @Override
    public void init() {

        demoServoRight = hardwareMap.servo.get("servo1");
        demoServoLeft = hardwareMap.servo.get("servo2");
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            demoServoRight.setPosition(closed1);
            demoServoLeft.setPosition(closed2);
        }
        if(gamepad1.b)
        {
            demoServoRight.setPosition(open1);
            demoServoLeft.setPosition(open2);
        }
        telemetry.addData("servo1", demoServoRight.getPosition());
        telemetry.addData("servo2", demoServoLeft.getPosition());
        telemetry.update();
    }
}