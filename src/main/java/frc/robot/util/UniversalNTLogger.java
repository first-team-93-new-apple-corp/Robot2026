// package frc.robot.util;

// import com.ctre.phoenix6.SignalLogger;
// import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import java.util.ArrayList;
// import java.util.List;
// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;
// import static edu.wpi.first.units.Units.*;

// /**
//  * Universal logger: register TalonFX devices (motor signals) and arbitrary
//  * numeric/boolean suppliers, and mirror the SmartDashboard table into
//  * SignalLogger + NetworkTables + SmartDashboard copies. Use flushAll() from
//  * Robot.periodic() or subsystem periodic to send data. Configurable cap
//  * prevents network/log overload.
//  */
// public class UniversalNTLogger {

//     private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
//     private final String namespace;
//     private final List<DeviceSignal> signals = new ArrayList<>();
//     private final List<BooleanSignal> boolSignals = new ArrayList<>();
//     private int maxSignalsPerFlush = 50;

//     private static class DeviceSignal {
//         final String name;
//         final DoubleSupplier supplier;
//         boolean enabled = true;
//         DeviceSignal(String name, DoubleSupplier supplier) { this.name = name; this.supplier = supplier; }
//     }

//     private static class BooleanSignal {
//         final String name;
//         final BooleanSupplier supplier;
//         boolean enabled = true;
//         BooleanSignal(String name, BooleanSupplier supplier) { this.name = name; this.supplier = supplier; }
//     }

//     public UniversalNTLogger(String namespace) {
//         this.namespace = namespace;
//         // Ensure CTRE SignalLogger is started once for the process
//         startSignalLoggerOnce();
//     }

//     private static volatile boolean signalLoggerStarted = false;

//     private static synchronized void startSignalLoggerOnce() {
//         if (!signalLoggerStarted) {
//             try {
//                 SignalLogger.start();
//             } catch (Throwable ignored) {
//                 // If SignalLogger.start() is not available or fails, ignore other
//                 // code will still publish to NetworkTables/SmartDashboard.
//             }
//             signalLoggerStarted = true;
//         }
//     }

//     public void setMaxSignalsPerFlush(int max) { this.maxSignalsPerFlush = Math.max(1, max); }

//     public void registerDouble(String name, DoubleSupplier supplier) {
//         signals.add(new DeviceSignal(name, supplier));
//     }

//     public void registerBoolean(String name, BooleanSupplier supplier) {
//         boolSignals.add(new BooleanSignal(name, supplier));
//     }

//     /**
//      * Convenience: register common TalonFX signals (velocity, position, currents, temp).
//      * Name is used as prefix, and signals will be published under "namespace/name/...".
//      */
//     public void registerTalonFX(String namePrefix, TalonFX motor) {
//         // velocity (RPS) - keep the unit choice consistent with your codebase
//         registerDouble(namePrefix + "/velocity", () -> {
//             try { return motor.getVelocity().getValue().in(edu.wpi.first.units.Units.RotationsPerSecond); }
//             catch (Throwable t) { return 0.0; }
//         });
//         // position (degrees or raw position)
//         registerDouble(namePrefix + "/position", () -> {
//             try { return motor.getPosition().getValue().in(Degrees); }
//             catch (Throwable t) { return 0.0; }
//         });
//         // stator current
//         registerDouble(namePrefix + "/statorCurrent", () -> {
//             try { return motor.getStatorCurrent().getValueAsDouble(); } catch (Throwable t) { return 0.0; }
//         });
//         // supply current
//         registerDouble(namePrefix + "/supplyCurrent", () -> {
//             try { return motor.getSupplyCurrent().getValueAsDouble(); } catch (Throwable t) { return 0.0; }
//         });
//         // NOTE: temperature and appliedOutput accessors vary by CTRE API; add them
//         // here if your version exposes them (example placeholders were removed).
//     }

//     /**
//      * Mirror everything currently under the SmartDashboard table to the SignalLogger
//      * and also publish to a mirror table under the configured namespace for visibility.
//      */
//     public void mirrorSmartDashboard() {
//         NetworkTable sd = ntInst.getTable("SmartDashboard");
//     java.util.Set<String> keys = sd.getKeys();
//     for (String key : keys) {
//             try {
//                 NetworkTableEntry e = sd.getEntry(key);
//                 if (e.getValue() == null) continue;
//                 String path = "SmartDashboard/" + key;
//                 if (e.getValue().isDouble() || e.getValue().isInteger()) {
//                     double v = e.getValue().getDouble();
//                     // write to mirror NetworkTable under namespace
//                     ntInst.getTable(namespace).getEntry(path).setDouble(v);
//                     SmartDashboard.putNumber(path, v);
//                     SignalLogger.writeDouble(path, v, "");
//                 } else if (e.getValue().isBoolean()) {
//                     boolean b = e.getValue().getBoolean();
//                     ntInst.getTable(namespace).getEntry(path).setBoolean(b);
//                     SmartDashboard.putBoolean(path, b);
//                     SignalLogger.writeBoolean(path, b);
//                 } else if (e.getValue().isString()) {
//                     String s = e.getValue().getString();
//                     ntInst.getTable(namespace).getEntry(path).setString(s);
//                     SmartDashboard.putString(path, s);
//                     // SignalLogger doesn't support strings; skip or write length
//                 } else if (e.getValue().isDoubleArray()) {
//                     double[] arr = e.getValue().getDoubleArray();
//                     ntInst.getTable(namespace).getEntry(path).setDoubleArray(arr);
//                     // SignalLogger has writeDoubleArray in CTRE; use it if available
//                     try { SignalLogger.writeDoubleArray(path, arr); } catch (Throwable ignored) {}
//                 }
//             } catch (Throwable ignored) {
//                 // ignore individual entry errors
//             }
//         }
//     }

//     /**
//      * Flush registered signals and mirror the SmartDashboard. Honors maxSignalsPerFlush
//      * to avoid saturating the network. Call from periodic().
//      */
//     public void flushAll() {
//         int published = 0;
//         for (DeviceSignal s : new ArrayList<>(signals)) {
//             if (!s.enabled) continue;
//             if (published >= maxSignalsPerFlush) break;
//             double v;
//             try { v = s.supplier.getAsDouble(); } catch (Throwable t) { v = 0.0; }
//             String path = namespace + "/" + s.name;
//             // publish to NetworkTables mirror
//             ntInst.getTable(namespace).getEntry(s.name).setDouble(v);
//             // update SmartDashboard mirror
//             SmartDashboard.putNumber(path, v);
//             // write to SignalLogger
//             try { SignalLogger.writeDouble(path, v, ""); } catch (Throwable ignored) {}
//             published++;
//         }

//         // Booleans after numeric signals (counts against same cap)
//         for (BooleanSignal b : new ArrayList<>(boolSignals)) {
//             if (!b.enabled) continue;
//             if (published >= maxSignalsPerFlush) break;
//             boolean val;
//             try { val = b.supplier.getAsBoolean(); } catch (Throwable t) { val = false; }
//             String path = namespace + "/" + b.name;
//             ntInst.getTable(namespace).getEntry(b.name).setBoolean(val);
//             SmartDashboard.putBoolean(path, val);
//             try { SignalLogger.writeBoolean(path, val); } catch (Throwable ignored) {}
//             published++;
//         }

//         // Mirror SmartDashboard table entries as well (optional heavy operation)
//         mirrorSmartDashboard();
//     }

//     public void setEnabled(String name, boolean enabled) {
//         for (DeviceSignal s : signals) if (s.name.equals(name)) s.enabled = enabled;
//         for (BooleanSignal b : boolSignals) if (b.name.equals(name)) b.enabled = enabled;
//     }
// }
