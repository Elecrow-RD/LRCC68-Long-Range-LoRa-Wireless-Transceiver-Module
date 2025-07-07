### 1, Product picture

![lora_transceiver_module_lr1262_01](F:\wiki\lorawan\LR1262 Long-Range LoRa Wireless Transceiver Module\git\lora_transceiver_module_lr1262_01.webp)

https://www.elecrow.com/lr1262-long-range-lora-wireless-transceiver-module-ultra-low-power-iot-industrial.html



### 2, Product version number

|      | Hardware | Software | Remark |
| ---- | -------- | -------- | ------ |
| 1    | V1.0     | V1.0     | latest |

### 3, product information

| Processor                   | None                                                         |
| :-------------------------- | :----------------------------------------------------------- |
| RF chip                     | Semtech SX1262 Chip                                          |
| LoRa Specification          |                                                              |
| Transmit Power TX           | 22 dBm@Max                                                   |
| Receiving Sensitivity       | -148dBm                                                      |
| Link Budget                 | Support 170db link budget                                    |
| Demodulator                 | Support LoRa/(G)FSK/GMSK signal modulation，SF5~SF12, Support 150MHz-960MHz configuration |
| Data Transfer Rate          | Supports data transmission rates from 0.018 to 62.5 kbps     |
| Other                       |                                                              |
| Frequency                   | Covering 850~930MHz (applicable to 868MHz, 915MHz)           |
| Communication Interface     | SPI(0~10Mbps)                                                |
| Lead Pin                    | Such as SPI signal, VCC, multiple GND, NRST, ANT, TX control, RX control, BUSY status, NFC, SWD debugging pin, etc. |
| Antenna                     | SMT pins, impedance @ 50 ohms                                |
| Reference Power Consumption | 1.62uA (sleep), 119mA (transmit + MCU), 6.8mA (receive + MCU) |
| Communication Distance      | 6~7km, up to 15km                                            |
| Crystal Type                | Industrial grade 32MHz TCXO                                  |
| Frequency Range             | 150 MHz to 960 MHz                                           |
| Operating Voltage           | 1.8~3.7V                                                     |
| Operating Temperature       | -40℃~+85℃, Industrial standard design                        |
| Package Size                | 10*10*3.5mm, 20-pin patch stamp hole design;                 |
| Net Weight                  | 1g                                                           |

### 4,Folder structure.

|--Datasheet: Includes datasheets for components used in the project, providing detailed specifications, electrical characteristics, and pin configurations.

|--example: Provides example code and projects to demonstrate how to use the hardware and libraries. These examples help users get started quickly.

|--factory_firmware: Stores pre-compiled factory firmware that can be directly flashed onto the device. This ensures the device runs the default functionality.

### 5,Pin definition

**IO port definition**

![LoRa_Transceiver_Module_LR1262_04](F:\wiki\lorawan\LR1262 Long-Range LoRa Wireless Transceiver Module\git\LoRa_Transceiver_Module_LR1262_04.webp)

##### Diagram:

![LoRa_Transceiver_Module_LR1262_06](F:\wiki\lorawan\LR1262 Long-Range LoRa Wireless Transceiver Module\git\LoRa_Transceiver_Module_LR1262_06.webp)