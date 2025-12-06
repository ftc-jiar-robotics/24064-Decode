package org.firstinspires.ftc.teamcode.decode.util;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; /**
 * Helper class for configuring the Brushland Labs Laser Rangefinder.
 * Online documentation: <a href="https://docs.brushlandlabs.com">...</a>
 */
public class LaserRangefinder {
    public final LynxI2cDeviceSynch i2c;

    public LaserRangefinder(RevColorSensorV3 device) {
        this.i2c = (LynxI2cDeviceSynch) device.getDeviceClient();
        this.i2c.enableWriteCoalescing(true);
    }

    /**
     * Set pin one to threshold for a range.
     */
    public void setPin0Digital(int thresh_lo, int thresh_hi) {
        setPin(R_PIN0_MODE, M_DIG, thresh_lo, thresh_hi);
    }

    public void setPin1Digital(int thresh_lo, int thresh_hi) {
        setPin(R_PIN1_MODE, M_DIG, thresh_lo, thresh_hi);
    }

    public void setPin0Analog(int bound_lo, int bound_hi) {
        setPin(R_PIN0_MODE, M_ANA, bound_lo, bound_hi);
    }

    public void setPin1Analog(int bound_lo, int bound_hi) {
        setPin(R_PIN1_MODE, M_ANA, bound_lo, bound_hi);
    }

    private void setPin(byte reg, byte reg2, int lo, int hi) {
        byte lo0 = (byte) (lo & 0xFF);
        byte lo1 = (byte) ((lo & 0xFF00) >> 8);
        byte hi0 = (byte) (hi & 0xFF);
        byte hi1 = (byte) ((hi & 0xFF00) >> 8);
        write(reg, new byte[]{reg2, lo0, lo1, hi0, hi1});
    }

    public int getPin0Mode() {
        return i2c.read8(R_PIN0_MODE);
    }

    public int getPin1Mode() {
        return i2c.read8(R_PIN1_MODE);
    }

    public void setDistanceMode(DistanceMode mode) {
        write(R_DISTMODE, new byte[]{(byte) (mode.ordinal() + 1)});
    }

    public DistanceMode getDistanceMode() {
        byte v = i2c.read8(R_DISTMODE);
        switch (v) {
            case 1: return DistanceMode.SHORT;
            case 2: return DistanceMode.MEDIUM;
            case 3: return DistanceMode.LONG;
            default: throw new RuntimeException("Could not get distance mode, " + v);
        }
    }

    /**
     * Set the timing budget and the total measurement period in milliseconds.
     * A period of zero means that the next range will begin immediately after the completion of the current range.
     */
    public void setTiming(int budget, int period) {
        if (budget < 5 || budget > 1000) throw new RuntimeException("Invalid timing budget: " + budget);
        if (period != 0 && (period < budget + 3)) throw new RuntimeException("Measurement period must be at least 4ms more than timing budget if set. For fast ranging, use a period of 0.");
        write(R_TIMING, new byte[]{(byte) (budget & 0xFF), (byte) ((budget & 0xFF00) >> 8), (byte) (period & 0xFF), (byte) ((period & 0xFF00) >> 8)});
    }

    public int[] getTiming() {
        java.nio.ByteBuffer buf = java.nio.ByteBuffer.wrap(i2c.read(R_TIMING, 4)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        return new int[]{buf.getShort(), buf.getShort()};
    }

    /**
     * Set the size of the sensor's region of interest. The ROI must be at least 4x4 in size.
     */
    public void setROI(int topLeftX, int topLeftY, int botRightX, int botRightY) {
        if (botRightX - topLeftX < 3 || topLeftY - botRightY < 3) throw new RuntimeException("ROI too small, must be at least 4x4.");
        for (int v : new int[]{topLeftX, topLeftY, botRightX, botRightY}) if (v < 0 || v > 15) throw new RuntimeException("Invalid ROI point value: " + v);
        write(R_ROI_TLX, new byte[]{(byte) topLeftX, (byte) topLeftY, (byte) botRightX, (byte) botRightY});
    }

    public int[] getROI() {
        byte[] data = i2c.read(R_ROI_TLX, 4);
        return new int[]{data[0], data[1], data[2], data[3]};
    }

    /**
     * Give the sensor a new I2C address from the default of 0x52.
     */
    public void setI2CAddress(int newAddress) {
        if (newAddress < 1 || newAddress > 127) throw new RuntimeException("Invalid I2C address: " + newAddress);
        write(R_IIC_ADDR, new byte[]{(byte) newAddress});
    }

    /**
     * Returns the (x, y) coordinates of the factory calibrated center of the sensor's 16x16 ROI.
     */
    public int[] getOpticalCenter() {
        byte[] data = i2c.read(R_OPTCENTERX, 2);
        return new int[]{data[0], data[1]};
    }

    /**
     * Resets from I2C scan mode to normal I2C mode, clearing previously configured scan ROIs.
     */
    public void setI2C() { // also clears custom i2c address
        write(R_PIN0_MODE, new byte[]{M_I2C, 0, 0, 0, 0});
    }

    public ScanSequenceBuilder setAnalogScanMode() {
        return new ScanSequenceBuilder(M_AN2);
    }

    public ScanSequenceBuilder setI2CScanMode() {
        return new ScanSequenceBuilder(M_II2);
    }

    public class ScanSequenceBuilder {
        private final byte mode;

        public ScanSequenceBuilder(byte mode) {
            this.mode = mode;
        }

        private final java.util.List<byte[]> rois = new java.util.ArrayList<>();

        public ScanSequenceBuilder addScanROI(
                int topLeftX,
                int topLeftY,
                int botRightX,
                int botRightY
        ) {
            rois.add(new byte[]{(byte) topLeftX, (byte) topLeftY, (byte) botRightX, (byte) botRightY});
            return this;
        }

        public void setScanROIs() {
            for (byte[] roi : rois) i2c.write(R_PIN0_MODE, new byte[]{mode, roi[0], roi[1], roi[2], roi[3]});
            write(R_PIN0_MODE, new byte[]{mode, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF});
        }
    }

    private int status = 0;

    /**
     * Retrieve the status of the last reading, where 0 indicates the reading is good, 1-2 indicates okay,
     * larger numbers indicates bad readings.
     */
    public int getStatus() {
        return status;
    }

    /**
     * Read distance via I2C. This also populates the value returned from {@link LaserRangefinder#getStatus}
     */
    public double getDistance(DistanceUnit unit) {
        byte[] data = i2c.read(R_PS_DATA_0, 2);
        status = (data[1] & 0xE0) >> 5;
        data[1] &= 0x1F;
        return unit.fromUnit(DistanceUnit.MM, java.nio.ByteBuffer.wrap(data, 0, 2)
                .order(java.nio.ByteOrder.LITTLE_ENDIAN)
                .getShort()
        );
    }

    private int b3 = 0;

    /**
     * In I2C scan mode, this returns the index of the ROI of the last reading from
     * {@link LaserRangefinder#getDistance(DistanceUnit)}.
     */
    public int getROINum() {
        return b3;
    }

    public double getScanDistance(DistanceUnit unit) {
        byte[] data = i2c.read(R_PS_DATA_0, 3);
        b3 = data[2];
        status = data[1] & 0xE0;
        data[1] &= 0x1F;
        return unit.fromUnit(DistanceUnit.MM, java.nio.ByteBuffer.wrap(data, 0, 2)
                .order(java.nio.ByteOrder.LITTLE_ENDIAN)
                .getShort()
        );
    }

    private void write(int creg, byte[] bytes) {
        i2c.write(creg, bytes);
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public enum DistanceMode {
        /**
         * Maximum range of 1.3m, good immunity to ambient light.
         */
        SHORT,
        /**
         * Maximum range of 3m, okay immunity to ambient light.
         */
        MEDIUM,
        /**
         * Maximum range of 4m, more susceptible to ambient light.
         */
        LONG
    }

    private static final byte M_I2C = 0;
    private static final byte M_ANA = 1;
    private static final byte M_DIG = 2;
    private static final byte M_AN2 = 3;
    private static final byte M_II2 = 4;

    private static final byte R_PS_DATA_0 = 0x08;
    private static final byte R_PIN0_MODE = 0x28;
    private static final byte R_PIN1_MODE = 0x2D;
    private static final byte R_ROI_TLX = 0x32;
    private static final byte R_DISTMODE = 0x36;
    private static final byte R_TIMING = 0x37; // timing 0
    private static final byte R_IIC_ADDR = 0x3B;
    private static final byte R_OPTCENTERX = 0x3C;
}
