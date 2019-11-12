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

    public int getTime() {
        return time;
    }

    int time = 0;

    public void DriveForward(double speed, double distance) {
        trobot.getLeftDriveFront().setPower(0.5);
        trobot.getRightDriveFront().setPower(0.5);
        trobot.getLeftDriveBack().setPower(0.5);
        trobot.getRightDriveBack().setPower(0.5);

        time = (int)((distance/(72.5*speed))*1000);

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

    public void StrafeRight() {
        trobot.getLeftDriveFront().setPower(-0.5);
        trobot.getRightDriveFront().setPower(0.5);
        trobot.getLeftDriveBack().setPower(0.5);
        trobot.getRightDriveBack().setPower(-0.5);
    }

    public void StrafeLeft() {
        trobot.getLeftDriveFront().setPower(0.5);
        trobot.getRightDriveFront().setPower(-0.5);
        trobot.getLeftDriveBack().setPower(-0.5);
        trobot.getRightDriveBack().setPower(0.5);
    }

    public void Stop() {
        trobot.getLeftDriveFront().setPower(0);
        trobot.getRightDriveFront().setPower(0);
        trobot.getLeftDriveBack().setPower(0);
        trobot.getRightDriveBack().setPower(0);
    }

    public void Latch() {
        trobot.getLeftServo().setPosition(0.5);
        trobot.getRightServo().setPosition(0.3);
    }

    public void Unlatch() {
        trobot.getLeftServo().setPosition(1);
        trobot.getRightServo().setPosition(0);
    }

    public void Collector(boolean in, boolean stop) {
        if(in) {
            trobot.getIntakeLeft().setPower(0.5);
            trobot.getIntakeRight().setPower(0.5);
        }
        else if(!in && !stop) {
            trobot.getIntakeLeft().setPower(0.2);
            trobot.getIntakeRight().setPower(0.2);
        } else if(stop) {
            trobot.getIntakeLeft().setPower(0);
            trobot.getIntakeRight().setPower(0);
        }
    }
}