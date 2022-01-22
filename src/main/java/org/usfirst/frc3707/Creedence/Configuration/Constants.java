package org.usfirst.frc3707.Creedence.Configuration;

public class Constants {

    public enum RobotType {
        CompetitionRobot, PracticeRobot
    }

    // Default to Competition Robot configuration
    public static RobotType robotType = RobotType.PracticeRobot;

    // public static void setRobotType(RobotType newRobotType) {
    // robotType = newRobotType;
    // }

    public static class HumanInputs {
        public static class DriverInputs {

        }

        public static class OperatorInputs {

        }

    }

    public static class PneumaticSystem {
        private static int pcmCANID = 1;

        public static int getPcmCanId() {
            return pcmCANID;
        }
    }

    public static class DriveSystem {


        public static class DriveModule {
            // NOTE: This class is declared static. This means it is a "nested class". It
            // cannot access any particular instance of the enclosing class (DriveSystem).
            // in this case, this does not matter, because there are no instances of DriveSystem
            // anyway.

            // All of the variables in this class, however, must be non-static. This is because
            // there will be multiple instances of this class within the DriveSystem class.
            private int drive_CompBot;
            private int swerve_CompBot;
            private int encoder_CompBot;
        
            private int drive_PracticeBot;
            private int swerve_PracticeBot;
            private int encoder_PracticeBot;
        
            private double offset_CompBot;
            private double offset_PracticeBot;
        
            //private RobotType robotType;
        
            public void setCompBotDrive(int drive) {
                this.drive_CompBot = drive;
            }
        
            public void setCompBotSwerve(int swerve) {
                this.swerve_CompBot = swerve;
            }
        
            public void setCompBotEncoder(int encoder) {
                this.encoder_CompBot = encoder;
            }
        
            public void setPracticeBotDrive(int drive) {
                this.drive_PracticeBot = drive;
            }
        
            public void setPracticeBotSwerve(int swerve) {
                this.swerve_PracticeBot = swerve;
            }
        
            public void setPracticeBotEncoder(int encoder) {
                this.encoder_PracticeBot = encoder;
            }
        
            public void setCompBotOffset(double offset) {
                this.offset_CompBot = offset;
            }
        
            public void setPracticeBotOffset(double offset) {
                this.offset_PracticeBot = offset;
            }
        
            public int getDrive() {
                if (robotType == RobotType.CompetitionRobot) {
                    return drive_CompBot;
                } else if (robotType == RobotType.PracticeRobot) {
                    return drive_PracticeBot;
                } else {
                    return -1;
                }
            }
        
            public int getSwerve() {
                if (robotType == RobotType.CompetitionRobot) {
                    return swerve_CompBot;
                } else if (robotType == RobotType.PracticeRobot) {
                    return swerve_PracticeBot;
                } else {
                    return -1;
                }
            }
        
            public int getEncoder() {
                if (robotType == RobotType.CompetitionRobot) {
                    return encoder_CompBot;
                } else if (robotType == RobotType.PracticeRobot) {
                    return encoder_PracticeBot;
                } else {
                    return -1;
                }
            }
        
            public double getOffset() {
                if (robotType == RobotType.CompetitionRobot) {
                    return offset_CompBot;
                } else if (robotType == RobotType.PracticeRobot) {
                    return offset_PracticeBot;
                } else {
                    return -1;
                }
            }
        }

        public static DriveModule FrontLeft = new DriveModule();
        public static DriveModule FrontRight = new DriveModule();
        public static DriveModule BackLeft = new DriveModule();
        public static DriveModule BackRight = new DriveModule();

        static {
            // Front Left - Competition Robot
            FrontLeft.setCompBotDrive(19);
            FrontLeft.setCompBotSwerve(18);
            FrontLeft.setCompBotEncoder(0);
            // Front Left - Practice Robot
            FrontLeft.setPracticeBotDrive(19);
            FrontLeft.setPracticeBotSwerve(18);
            FrontLeft.setPracticeBotEncoder(0);

            // Front Right - Competition Robot
            FrontRight.setCompBotDrive(17);
            FrontRight.setCompBotSwerve(16);
            FrontRight.setCompBotEncoder(3);
            // Front Left - Practice Robot
            FrontRight.setPracticeBotDrive(17);
            FrontRight.setPracticeBotSwerve(16); // 5?
            FrontRight.setPracticeBotEncoder(3);

            // Back Left - Competition Robot
            BackLeft.setCompBotDrive(13);
            BackLeft.setCompBotSwerve(12);
            BackLeft.setCompBotEncoder(1);
            BackLeft.setCompBotOffset(276);
            // Back Left - Practice Robot
            BackLeft.setPracticeBotDrive(13);
            BackLeft.setPracticeBotSwerve(12);
            BackLeft.setPracticeBotEncoder(1);

            // Back Right - Competition Robot
            BackRight.setCompBotDrive(15);
            BackRight.setCompBotSwerve(14);
            BackRight.setCompBotEncoder(2);
            // Back Right - Practice Robot
            BackRight.setPracticeBotDrive(15);
            BackRight.setPracticeBotSwerve(14);
            BackRight.setPracticeBotEncoder(2);
            
            //COMP BOT OFFSETS
            FrontLeft.setCompBotOffset(60);
            FrontRight.setCompBotOffset(28);
            BackLeft.setCompBotOffset(11);
            BackRight.setCompBotOffset(18);

            //PRACTIVE BOT OFFSETS
            FrontLeft.setPracticeBotOffset(197);
            FrontRight.setPracticeBotOffset(192);
            BackLeft.setPracticeBotOffset(207);
            BackRight.setPracticeBotOffset(241);

        }
    }

    public static class CargoDeliverySystem{

        private static int LeftMotorControllerPWMChannel = 10;
        private static int RightMotorControllerPWMChannel = 11;

        private static boolean LeftMotorControllerInverted_CompBot = false;
        private static boolean LeftMotorControllerInverted_PracticeBot = false;

        private static boolean RightMotorControllerInverted_CompBot = false;
        private static boolean RightMotorControllerInverted_PracticeBot = false;

        public static int getLeftMotorControllerPWMChannel() {
            return LeftMotorControllerPWMChannel;
        }

        public static int getRightMotorControllerPWMChannel() {
            return RightMotorControllerPWMChannel;
        }

        public static boolean getLeftMotorControllerInverted() {
            if (robotType == RobotType.CompetitionRobot) {
                return LeftMotorControllerInverted_CompBot;
            } else {
                return LeftMotorControllerInverted_PracticeBot;
            }
        }

        public static boolean getRightMotorControllerInverted() {
            if (robotType == RobotType.CompetitionRobot) {
                return RightMotorControllerInverted_CompBot;
            } else {
                return RightMotorControllerInverted_PracticeBot;
            }
        }

    }

    public static class CargoIntakeSystem {

        private static int MotorControllerPWMChannel = 6;
        
        private static boolean MotorControllerInverted_CompBot = false;
        private static boolean MotorControllerInverted_PracticeBot = false;

        private static int tongueSolenoidForward = 4;
        private static int tongueSolenoidReverse = 5;


        public static int getMotorControllerPWMChannel() {
            return MotorControllerPWMChannel;
        }

        public static boolean getMotorControllerInverted() {
            if (robotType == RobotType.CompetitionRobot) {
                return MotorControllerInverted_CompBot;
            } else {
                return MotorControllerInverted_PracticeBot;
            }
        }

        public static int getTongueSolenoidForward() {
            return tongueSolenoidForward;
        }

        public static int getTongueSolenoidReverse() {
            return tongueSolenoidReverse;
        }
    }

    public static class HatchSystem {
        private static int clawButton1 = 1;
        private static int clawButton2 = 2;

        private static int clampSolenoidForward = 2;
        private static int clampSolenoidReverse = 3;

        private static int thrustSolenoidForward = 0;
        private static int thrustSolenoidReverse = 1;

        public static int getClawButton1() {
            return clawButton1;
        }

        public static int getClawButton2() {
            return clawButton2;
        }

        public static int getClampSolenoidForward() {
            return clampSolenoidForward;
        }

        public static int getClampSolenoidReverse() {
            return clampSolenoidReverse;
        }

        public static int getThrustSolenoidForward() {
            return thrustSolenoidForward;
        }

        public static int getThrustSolenoidReverse() {
            return thrustSolenoidReverse;
        }
    }

    public static class ClimbSystem {
        private static int climbBar = 8;
        private static int climbGrab = 7;
        private static int climbPullForward = 4;

        public static int getClimbBar() {
            return climbBar;
        }

        public static int getClimbGrab() {
            return climbGrab;
        }

        public static int getClimbPullForward() {
            return climbPullForward;
        }
    }

    public static class LiftSystem {
        private static int pneumaticLiftForward = 4;
        private static int pneumaticLiftReverse = 5;

        public static int getPneumaticLiftForward() {
            return pneumaticLiftForward;
        }

        public static int getPneumaticLiftReverse() {
            return pneumaticLiftReverse;
        }

    }

    public static class CameraSystem{
        private static int frontCamera_CompBot = 0;
        private static int frontCamera_PracticeBot = 0;

        private static int rearCamera_CompBot = 1;
        private static int rearCamera_PracticeBot = 1;

        private static int frontCameraResolutionX = 160;
        private static int frontCameraResolutionY = 120;
        private static int frontCameraFPS = 30;

        private static int rearCameraResolutionX = 160;
        private static int rearCameraResolutionY = 120;
        private static int rearCameraFPS = 30;

        public static int getFrontCamera(){
            if (robotType == RobotType.CompetitionRobot) {
                return frontCamera_CompBot;
            } else if (robotType == RobotType.PracticeRobot) {
                return frontCamera_PracticeBot;
            } else {
                return -1;
            }
        }
        public static int getRearCamera(){
            if (robotType == RobotType.CompetitionRobot) {
                return rearCamera_CompBot;
            } else if (robotType == RobotType.PracticeRobot) {
                return rearCamera_PracticeBot;
            } else {
                return -1;
            }
        }

        public static int getFrontCameraResolutionX(){
            return frontCameraResolutionX;
        }
        public static int getFrontCameraResolutionY(){
            return frontCameraResolutionY;
        }
        public static int getFrontCameraFPS(){
            return frontCameraFPS;
        }

        public static int getRearCameraFPS(){
            return rearCameraFPS;
        }

        public static int getRearCameraResolutionX(){
            return rearCameraResolutionX;
        }
        public static int getRearCameraResolutionY(){
            return rearCameraResolutionY;
        }


    }
    // the following should be placed into sub-classes of Constants
    public static int Grabber = 7;
    public static int Winch = 8;
    public static int Elevator = 9;
    public static int lidar1 = 0;

}