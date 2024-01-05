package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Clow", group = "TeleOp")
public class ServoControl extends OpMode {

    private Servo demoServo;
    private Servo demoServo2;


    @Override
    public void init() {
        demoServo = hardwareMap.servo.get("servo");
        demoServo2 = hardwareMap.servo.get("servo2");
    }

    @Override
    public void loop() {

        demoServo = hardwareMap.servo.get("servo");                                         // 2 servos,
        demoServo2 = hardwareMap.servo.get("servo2");

        // Add any additional logic here...
        if (gamepad1.right_trigger > 0.1) {  //strange
                demoServo.setPosition(demoServo.getPosition() - 0.2);
        }
        // Check if LT (Left Trigger) is pressed to set the servo to min position
        else if (gamepad1.right_bumper != false) {     //restranghe
                demoServo.setPosition(demoServo.getPosition() + 0.2);
        }

        if (gamepad1.left_trigger > 0.1) {  //strange
                demoServo2.setPosition(demoServo2.getPosition() + 0.2);
        }
        // Check if LT (Left Trigger) is pressed to set the servo to min position
        else if (gamepad1.left_bumper != false) {     //restranghe
                demoServo2.setPosition(demoServo2.getPosition() - 0.2);
        }
        telemetry.addData("RT Pressed", gamepad1.right_trigger > 0.1);
        telemetry.addData("RB Pressed", gamepad1.right_bumper == true );
        telemetry.addData("LT Pressed", gamepad1.left_trigger > 0.1);
        telemetry.addData("LB Pressed", gamepad1.left_bumper == true );
        //telemetry.update();
    }
}
