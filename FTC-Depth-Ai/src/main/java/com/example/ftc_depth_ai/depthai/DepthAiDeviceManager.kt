package com.example.ftc_depth_ai.depthai

import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice
import com.qualcomm.robotcore.util.SerialNumber
import org.firstinspires.ftc.robotcore.internal.ftdi.VendorAndProductIds

class DepthAiDeviceManager{
    companion object {
        private val mInstance: DepthAiDeviceManager = TODO()

//        @JvmStatic fun getInstance(): DepthAiDeviceManager {
//            return mInstance
//        }
        val supportedDevices: List<VendorAndProductIds> = arrayOf(
            VendorAndProductIds(0x03E7, 0x0) // Generic Luxonis Depth Ai Oak Camera Product
        ).asList()

        val usbManager: RobotUsbManagerDepthAi
    }
    fun getInstance(): DepthAiDeviceManager {
        return mInstance
    }
    fun scanForDevices(): MutableList<SerialNumber> {
        TODO("Not yet implemented")
    }

    fun openBySerialNumber(serialNumber: SerialNumber?): RobotUsbDevice {
        TODO("Not yet implemented")
    }

    fun createDeviceInfoList(): Int {
        TODO()
    }

    fun getDeviceInfoListDetail(index: Int): DepthAiDevice.DeviceInfo {
        TODO()
    }
}
