//package frc.robot.subsystems;
//
//public class Example {
//    public static class ClearWaterBottle {
//        public void open() {
//            System.out.println("Opening bottle");
//        }
//
//        public void close() {
//            System.out.println("Closing bottle");
//        }
//
//        public void drink() {
//            System.out.println("Drinking from bottle");
//        }
//    }
//
//    public static class OpaqueWaterBottle {
//        public void open() {
//            System.out.println("Opening bottle");
//        }
//
//        public void close() {
//            System.out.println("Closing bottle");
//        }
//
//        public void drink() {
//            System.out.println("Drinking from bottle");
//        }
//    }
//
//    // Tow Brothers House
//    // Door (10, 10)
//    //
//
//    // Tow Brothers - large open warehouses, the same kind
//    // Door at coordinates (50, 50)
//    // Meeting Room at coordinates (25, 25)
//    // Roof - (0,0), area (50, 50)
//
//    // Sock Factory from JCPenny - (50,50)
//    // Sock Factory from Talbots - (50, 50)
//    // Sock Factory from TJMax - (25, 25)
//
//
//    public static void openAndCloseBottle(ClearWaterBottle bottle) {
//        bottle.open();
//        bottle.close();
//    }
//
//    public static void main(String[] args) {
//        openAndCloseBottle(new OpaqueWaterBottle());
//    }
//
//    public Coordinates createCoordinates() {
//        Coordinates coordinates = new Coordinates();
//        coordinates.setX(1.0);
//        coordinates.setY(1.0);
//        return coordinates;
//    }
//
//    public Coordinates betterCreateCoordinates() {
//        return new Coordinates().withX(2.0).withY(1.0);
//    }
//
//
//    public static class TowBrothersWarehouse {
//        public boolean hasDoorAt5050() {
//            return true;
//        }
//
//        public void set5050DoorOpen(boolean open) {
//
//        }
//
//        public void destroyRoof() {
//
//        }
//
//        public void reinstateRoof() {}
//    }
//
//    public static class JcPennySockFactory extends TowBrothersWarehouse {
//        public JcPennySockFactory() {
//
//        }
//
//        @Override
//        public boolean hasDoorAt5050() {
//            return false;
//        }
//    }
//}
