package com.example.ftc_depth_ai.depthai

import com.qualcomm.robotcore.exception.RobotCoreException
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice
import com.qualcomm.robotcore.hardware.usb.RobotUsbManager
import com.qualcomm.robotcore.util.SerialNumber

class RobotUsbManagerDepthAi : RobotUsbManager
{
    val deviceManager: DepthAiDeviceManager = DepthAiDeviceManager().getInstance()
    
    var numberOfDevices = 0

    @Synchronized
    @Throws(RobotCoreException::class)
    override fun scanForDevices(): List<SerialNumber>? {
        numberOfDevices = deviceManager.createDeviceInfoList()
        val result: MutableList<SerialNumber> = ArrayList<SerialNumber>(numberOfDevices)
        for (i in 0 until numberOfDevices) {
            getDeviceSerialNumberByIndex(i)?.let { result.add(it) }
        }
        return result
    }

    @Throws(RobotCoreException::class)
    protected fun getDeviceSerialNumberByIndex(index: Int): SerialNumber? {
        return SerialNumber.fromString(deviceManager.getDeviceInfoListDetail(index).serialNumber)
    }

    override fun openBySerialNumber(serialNumber: SerialNumber?): RobotUsbDevice {
        TODO("Not yet implemented")
    }
}