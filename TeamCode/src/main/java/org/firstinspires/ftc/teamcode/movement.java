package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "movement", group = "TeleOp")
public class movement extends OpMode {

    DcMotor motor2;
    DcMotor motor1;

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor1"); // Replace "motor1" with the name of your motor
        motor2 = hardwareMap.dcMotor.get("motor2"); // Replace "motor2" with the name of your motor
    }

    @Override
    public void loop() {
        double leftStickX = gamepad1.left_stick_y; // Negate the value because the joystick is inverted
        double leftStickY = gamepad1.left_stick_x;

        double motor2Power = leftStickY + leftStickX;
        double motor1Power = leftStickY - leftStickX;

        motor1.setPower(-motor1Power);
        motor2.setPower(-motor2Power);
        if(gamepad1.y){
            move_f();
        }
        if(gamepad1.x){
            move_l();
        }
        if(gamepad1.b){
            move_r();
        }
        if(gamepad1.a){
            move_b();
        }
    }
    public void move_f(){
        motor1.setPower(-1);
        motor2.setPower(1);
    }
    public void move_b(){
        motor1.setPower(1);
        motor2.setPower(-1);
    }
    public void move_l(){
        motor1.setPower(1);
        motor2.setPower(1);
    }
    public void move_r(){
        motor1.setPower(-1);
        motor2.setPower(-1);
    }
}
