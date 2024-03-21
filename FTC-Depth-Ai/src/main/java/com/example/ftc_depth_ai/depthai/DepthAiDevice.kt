package com.example.ftc_depth_ai.depthai

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.util.SerialNumber

@DeviceProperties(xmlTag = "DepthAiCamera", name = "Depth Ai Camera")
class DepthAiDevice : HardwareDevice {
    lateinit var serialNumber: SerialNumber

    private val mDeviceName = "nooo"

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Other

    override fun getDeviceName(): String = mDeviceName

    override fun getConnectionInfo(): String {
        var string = deviceName
        string+="; Connected at "

        return string
    }

    override fun getVersion(): Int {
        TODO("Not yet implemented")
    }

    override fun resetDeviceConfigurationForOpMode() {
        TODO("Not yet implemented")
    }

    override fun close() {
        TODO("Not yet implemented")
    }

    data class DeviceInfo(
        var flags: Int = 0,
        var bcdDevice: Short = 0,
        var type: Int = 0,
        var iSerialNumber: Byte = 0x0,
        var id: Int = 0,
        var location: Int = 0,
        var serialNumber: String? = null,
        var manufacturuer: String = "Luxonis",
        var productName: String = "GENERIC DEPTHAI CAMERA",
        var handle: Int = 0,
        var breakOnParam: Int = 0,
        var modemStatus: Short = 0,
        var lineStatus: Short = 0
    )
}