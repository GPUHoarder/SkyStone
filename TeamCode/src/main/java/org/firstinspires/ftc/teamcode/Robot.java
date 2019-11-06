package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Robot {
    private DcMotor intakeRight = null;
    private DcMotor intakeLeft = null;

    private DcMotor leftDriveFront = null;
    private DcMotor rightDriveFront = null;
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;

    private Servo rightServo = null;
    private Servo leftServo = null;

    public DcMotor getIntakeRight() {
        return intakeRight;
    }

    public void setIntakeRight(DcMotor intakeRight) {
        this.intakeRight = intakeRight;
    }

    public DcMotor getIntakeLeft() {
        return intakeLeft;
    }

    public void setIntakeLeft(DcMotor intakeLeft) {
        this.intakeLeft = intakeLeft;
    }

    public DcMotor getLeftDriveFront() {
        return leftDriveFront;
    }

    public void setLeftDriveFront(DcMotor leftDriveFront) {
        this.leftDriveFront = leftDriveFront;
    }

    public DcMotor getRightDriveFront() {
        return rightDriveFront;
    }

    public void setRightDriveFront(DcMotor rightDriveFront) {
        this.rightDriveFront = rightDriveFront;
    }

    public DcMotor getLeftDriveBack() {
        return leftDriveBack;
    }

    public void setLeftDriveBack(DcMotor leftDriveBack) {
        this.leftDriveBack = leftDriveBack;
    }

    public DcMotor getRightDriveBack() {
        return rightDriveBack;
    }

    public void setRightDriveBack(DcMotor rightDriveBack) {
        this.rightDriveBack = rightDriveBack;
    }

    public Servo getRightServo() {
        return rightServo;
    }

    public void setRightServo(Servo rightServo) {
        this.rightServo = rightServo;
    }

    public Servo getLeftServo() {
        return leftServo;
    }

    public void setLeftServo(Servo leftServo) {
        this.leftServo = leftServo;
    }
}
