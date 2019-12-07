package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Trobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.Math;

public class Drive {
    private Trobot trobot = null;

    Drive(Trobot trobot) {
        this.trobot = trobot;
    }

    public int getTime() {
        return time;
    }

    private int time = 0;

    public void driving(double leftPower, double rightPower) {
        trobot.getLeftDriveFront().setPower(leftPower);
        trobot.getRightDriveFront().setPower(rightPower);
        trobot.getLeftDriveBack().setPower(leftPower);
        trobot.getRightDriveBack().setPower(rightPower);
    }

    public void straightDrive(int direction) {
        trobot.getLeftDriveFront().setPower(0.5*direction);
        trobot.getRightDriveFront().setPower(0.5*direction);
        trobot.getLeftDriveBack().setPower(0.5*direction);
        trobot.getRightDriveBack().setPower(0.5*direction);
    }

    public void encoderDrive(double speed, double distance) {

        double threadsPerCentimeter = ((1120*2)/(10*3.1415));

        trobot.getLeftDriveFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trobot.getRightDriveFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trobot.getLeftDriveBack().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trobot.getRightDriveBack().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        trobot.getLeftDriveFront().setTargetPosition((int)(distance * threadsPerCentimeter));
        trobot.getRightDriveFront().setTargetPosition((int)(distance * threadsPerCentimeter));
        trobot.getLeftDriveBack().setTargetPosition((int)(distance * threadsPerCentimeter));
        trobot.getRightDriveBack().setTargetPosition((int)(distance * threadsPerCentimeter));

        trobot.getLeftDriveFront().setPower(speed);
        trobot.getRightDriveFront().setPower(speed);
        trobot.getLeftDriveBack().setPower(speed);
        trobot.getRightDriveBack().setPower(speed);

        while (trobot.getLeftDriveFront().isBusy() && trobot.getRightDriveBack().isBusy()) {
        }

        trobot.getLeftDriveFront().setPower(0);
        trobot.getRightDriveFront().setPower(0);
        trobot.getLeftDriveBack().setPower(0);
        trobot.getRightDriveBack().setPower(0);
    }

    public void skyClaw(boolean down) {
        if(down) {
            trobot.getHookServo().setPosition(0.5);
        } else {
            trobot.getHookServo().setPosition(0);
        }
    }

    public void move(double speed, double distance) {
        if (speed > 1) {
            speed = 1;
        }
        if (distance > 0) {
            trobot.getLeftDriveFront().setPower(speed);
            trobot.getRightDriveFront().setPower(speed);
            trobot.getLeftDriveBack().setPower(speed);
            trobot.getRightDriveBack().setPower(speed);
        } else if (distance < 0) {
            trobot.getLeftDriveFront().setPower(-speed);
            trobot.getRightDriveFront().setPower(-speed);
            trobot.getLeftDriveBack().setPower(-speed);
            trobot.getRightDriveBack().setPower(-speed);
        }


        time = Math.abs((int)((distance/(72.5*speed))*1000));

    }


    public void turnRight(double degrees) {

    }

    public void turnLeft(double degrees) {

    }

    public void strafeRight() {
        trobot.getLeftDriveFront().setPower(-1);
        trobot.getRightDriveFront().setPower(1);
        trobot.getLeftDriveBack().setPower(1);
        trobot.getRightDriveBack().setPower(-1);
    }

    public void strafeLeft() {
        trobot.getLeftDriveFront().setPower(1);
        trobot.getRightDriveFront().setPower(-1);
        trobot.getLeftDriveBack().setPower(-1);
        trobot.getRightDriveBack().setPower(1);
    }

    public void stop() {
        trobot.getLeftDriveFront().setPower(0);
        trobot.getRightDriveFront().setPower(0);
        trobot.getLeftDriveBack().setPower(0);
        trobot.getRightDriveBack().setPower(0);
    }

   // public void latch() {
     //   trobot.getLeftServo().setPosition(0.5);
       // trobot.getRightServo().setPosition(0.3);
    //}

    /*public void unlatch() {
        trobot.getLeftServo().setPosition(1);
        trobot.getRightServo().setPosition(0);
    }*/

    /*public void collector(boolean in, boolean stop) {
        if(in) {
            trobot.getIntakeLeft().setPower(-0.5);
            trobot.getIntakeRight().setPower(-0.5);
        }
        else if(!in && !stop) {
            trobot.getIntakeLeft().setPower(0.2);
            trobot.getIntakeRight().setPower(0.2);
        } else if(stop) {
            trobot.getIntakeLeft().setPower(0);
            trobot.getIntakeRight().setPower(0);
        }
    }*/
}