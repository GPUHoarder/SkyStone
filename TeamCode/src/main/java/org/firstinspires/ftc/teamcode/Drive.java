package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Trobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Drive {
    private Trobot trobot = null;

    Drive(Trobot trobot) {
        this.trobot = trobot;
    }


    public void DriveForward(double distance) {

        trobot.getLeftDriveFront().setPower(0.5);
        trobot.getRightDriveFront().setPower(0.5);
        trobot.getLeftDriveBack().setPower(0.5);
        trobot.getRightDriveBack().setPower(0.5);

//        double actualSpeed = (72.5 * s);
//        sleep((int)(d/actualSpeed * 1000));
//


    }

    public void DriveBackward(double speed) {
        trobot.getLeftDriveFront().setPower(-speed);
        trobot.getRightDriveFront().setPower(-speed);
        trobot.getLeftDriveBack().setPower(-speed);
        trobot.getRightDriveBack().setPower(-speed);


    }

    public void TurnRight(double degrees) {

    }

    public void TurnLeft(double degrees) {

    }

    public void StrafeRight(double distance, double speed) {

    }

    public void StrafeLeft(double distance, double speed) {


    }
    public void Stop() {
        trobot.getLeftDriveFront().setPower(0);
        trobot.getRightDriveFront().setPower(0);
        trobot.getLeftDriveBack().setPower(0);
        trobot.getRightDriveBack().setPower(0);
    }
}