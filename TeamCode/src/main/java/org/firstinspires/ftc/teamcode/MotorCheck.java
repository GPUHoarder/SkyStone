package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Test", group="Linear OpMode")

public class MotorCheck extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;

    private DcMotor leftSucc = null;
    private DcMotor rightSucc = null;

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive2  = hardwareMap.get(DcMotor.class, "left_drive2");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive2");
        rightSucc  = hardwareMap.get(DcMotor.class, "rightSucc");
        leftSucc = hardwareMap.get(DcMotor.class, "leftSucc");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        leftDrive.setPower(0.6);
        rightDrive.setPower(0.6);
        leftDrive2.setPower(0.6);
        rightDrive2.setPower(0.6);
        leftSucc.setPower(0.6);
        rightSucc.setPower(0.6);

        sleep(1000);

        leftDrive.setPower(-0.6);
        rightDrive.setPower(-0.6);
        leftDrive2.setPower(-0.6);
        rightDrive2.setPower(-0.6);
        leftSucc.setPower(-0.6);
        rightSucc.setPower(-0.6);


    }

}