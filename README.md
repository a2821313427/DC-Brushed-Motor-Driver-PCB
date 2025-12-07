
# **直流有刷电机驱动板项目 **

## **项目概述 **

一个用于驱动 12V 直流有刷电机的控制板。
电路输入 12V 电源，并由 STM32 主控芯片输出 PWM 波形，从而控制电机的转速与方向。
**项目从原理图设计、PCB排布、焊接与调试，均由本人完成。
这是我完成的首个 PCB 项目，在陈老师的帮助下完成，特此致谢。**

## **系统架构 **

### **1. 电源系统**

* 为电机提供 12V 电源。
* 使用稳压芯片将 12V 转换为 3.3V，供给 STM32 主控芯片。
* 使用基准电压芯片提供 1.2V 电压给运算放大器。

### **2. 主控芯片（STM32F103C8T6）**

* 负责输出指定占空比的 PWM 波形。
* 支持电压、电流检测（ADC）。
* 通过 SWD 与电脑通信。
* 可通过 I²C 输出内容到显示屏。

### **3. H 桥驱动电路**

* MCU 输出的 PWM 波形经过缓冲器、半桥驱动器。
* 四个 NMOS 构成 H 桥，实现电机正反转与调速控制。

### **4. 电流与电压检测回路**

* 使用采样电阻检测电流。
* 电压由 MCU ADC 模块采样。
* 运算放大器完成信号放大。

### **5. 用户交互（按钮与指示灯）**

* 提供加速、减速、开关、方向切换等按钮。
* 使用两个 LED 显示当前运行模式。

---

## **功能说明 **

电机具有正转三档、反转三档。
LED 通过不同闪烁频率表示当前档位。

三个档位对应 PWM 占空比为：

* 30%
* 50%
* 70%

---

## **PCB 设计与制作 **

* 使用 Altium Designer 完成原理图与 PCB 设计。
* 在嘉立创打板并自行购买元件焊接。
* 所有调试均独立完成。

---

## **焊接与调试流程 **

1. 首先确认电源系统（12V→3.3V）正常工作。
2. 焊接最小系统并确认 MCU 可正常运行与通信。
3. 焊接 H 桥与其它外围电路。
4. 完成整机调试，确认运行成功。

---

## **项目文件 **

所有原理图、PCB 工程文件，以及 STM32 Keil 固件均存放在仓库 **master 分支** 中。
相关图片和视频可在**images**中查看。

---

---

# **DC Brushed Motor Driver PCB Project (My First PCB Design)**

## **Project Overview (English)**

This is my first completed PCB design — a driver board for a 12V DC brushed motor.
The system receives a 12V input and uses an STM32 microcontroller to generate PWM signals, enabling control of motor speed and rotation direction.
This project was completed under the guidance of **Professor Chen**, to whom I express my sincere gratitude.

## **System Architecture (English)**

### **1. Power System**

* Supplies 12V power to the motor.
* A voltage regulator converts 12V to 3.3V for the STM32 microcontroller.
* A voltage reference provides a stable 1.2V signal for the operational amplifier.

### **2. Main Controller (STM32F103C8T6)**

* Generates PWM signals with specific duty cycles.
* Supports voltage and current sensing through the ADC module.
* Communicates with a PC via SWD.
* Can output display data via the I²C interface.

### **3. H-Bridge Motor Driver Circuit**

* PWM signals pass through a buffer and a half-bridge driver.
* Four NMOS transistors form the H-bridge.
* Enables forward/reverse rotation and speed control.

### **4. Current and Voltage Sensing Circuit**

* A shunt resistor is used for current measurement.
* Voltage drop is sampled by the MCU ADC module.
* An op-amp provides signal amplification and conditioning.

### **5. User Interface (Buttons & LEDs)**

* Five buttons provide acceleration, deceleration, power control, and direction switching.
* Two LEDs indicate the motor's current operating mode.

---

## **Features (English)**

The motor supports three forward-speed levels and three reverse-speed levels.
LED blink frequency reflects the current speed mode.

PWM duty cycles for the three levels:

* 30%
* 50%
* 70%

---

## **PCB Design and Manufacturing (English)**

* Designed using Altium Designer (AD).
* Manufactured by JLCPCB; components were soldered manually.
* Debugging and validation were completed independently.

---

## **Assembly & Debugging Procedure (English)**

1. Verify the power system (12V→3.3V).
2. Assemble the MCU minimal system and confirm proper operation and communication.
3. Assemble the H-bridge and remaining circuitry.
4. Perform final debugging and functional testing.

---

## **Project Files (English)**
files, and STM32 Keil firmware are stored in the **master branch** of this repository.



