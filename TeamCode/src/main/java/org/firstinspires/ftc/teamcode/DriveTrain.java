package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "DriveTrain", group = "B")
public class DriveTrain extends LinearOpMode {
    DcMotor motor2;
    DcMotor motor1;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = (DcMotor)this.hardwareMap.dcMotor.get("motor1");
        motor2 = (DcMotor)this.hardwareMap.dcMotor.get("motor2");
        while(opModeIsActive()){
            double leftStickX = (double) this.gamepad2.left_stick_y;
            double leftStickY = (double) this.gamepad2.right_stick_x;
            double motor2Power = leftStickY + leftStickX;
            double motor1Power = leftStickY - leftStickX;
            this.motor1.setPower(-motor1Power);
            this.motor2.setPower(-motor2Power);
        }
    }
}
