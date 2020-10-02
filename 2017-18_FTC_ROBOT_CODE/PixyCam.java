package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cSensor(name = "CMUcam5 Pixy Camera", description = "Camera from Charmed Labs", xmlTag = "PixyCam")
public class PixyCam extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public enum Register {

        FIRST(0),
        SYNC_A(0x01),
        SYNC_B(0x02),
        CHECKSUM_A(0x03),
        CHECKSUM_B(0x04),
        SIGNATURE_NUMBER_A(0x05),
        SIGNATURE_NUMBER_B(0x06),
        X_COORDINATE_A(0x07),
        X_COORDINATE_B(0x08),
        Y_COORDINATE_A(0x09),
        Y_COORDINATE_B(0x10),
        WIDTH_A(0x11),
        WIDTH_B(0x12),
        HEIGHT_A(0x13),
        HEIGHT_B(0x14),
        LAST(HEIGHT_B.bVal);

        public int bVal;

        Register(int bVal){
            this.bVal = bVal;
        }
    }

    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    public PixyCam(I2cDeviceSynch deviceClient) {
        super(deviceClient,true);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x55));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.valueOf("Charmed Labs");
    }

    @Override
    public String getDeviceName() {
        return "CMUcam5 Pixy";
    }
}
