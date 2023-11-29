package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp
public class PidControl extends OpMode{
    private PIDController controller;
    public static double p=0, i=0, d=0;
    public static double f=0;
    public static int target=0;
    private final double ticks_in_degree = 700 / 180.0;
    DcMotorEx arm1,arm2;
    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm1 = hardwareMap.get(DcMotorEx.class, "motor_arm1");
        arm2 = hardwareMap.get(DcMotorEx.class, "motor_arm2");
    }
    @Override
    public void loop(){
        controller.setPID(p,i,d);
        int armPos = arm1.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
        double power = pid + ff;
        arm1.setPower(power);
        arm2.setPower(-power);
        telemetry.addData("pos", armPos);
        telemetry.addData("target",target);
        telemetry.update();
    }
}