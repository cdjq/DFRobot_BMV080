# DFRobot_BMV080

- [English Version](./README.md)

这款 BMV080 PM2.5 传感器模块设计紧凑、精度高且测量范围广。其核心采用博世最新研发的 BMV080 传感器元件——全球最小的 PM 空气质量传感器。其体积比市场上同类产品小 450 多倍。尽管尺寸大幅缩减，但性能丝毫不减。它不仅能精确测量空气中 PM2.5 颗粒的质量浓度，还支持 PM1 和 PM10 的检测。<br>
传统的 PM2.5 传感器通常依靠风扇或风道将自由漂浮的颗粒引入检测区域，因此体积较大，并伴有风扇噪音和灰尘堆积问题，这增加了维护成本和故障风险。然而，这款传感器采用类似于相机的测量原理，运用激光光学技术，根据自由空间中颗粒的数量和相对速度来计算质量浓度。它巧妙地利用周围自然气流驱动颗粒进入检测区域进行直接测量，无需风扇或强制气流系统，从而消除了维护麻烦，避免了风扇造成的灰尘堆积，显著提高了设备的可靠性。<br>
![Fermion_BMV080](image/Fermion_BMV080.JPG)

## 产品链接
    SKU:XXXXX

## 目录

  * [概述](#概述)
  * [库安装](#库安装)
  * [方法](#方法)
  * [兼容性](#兼容性)
  * [历史](#历史)
  * [创作者](#创作者)

## 概述

DFRobot_BMV080 是一个为驱动博世新型空气质量测量芯片 BMV080 而设计的 Arduino 库。
BMV080 目前是世界上最小的 PM2.5 芯片，它采用激光进行测量。
此库提供了两种通信方式：IIC 和 SPI，以及一些基本的例程。目前，它仅支持 ESP32 平台。

## 库安装

要使用此库，请首先下载库文件，将其粘贴到“Arduino\libraries”目录中，然后打开“示例”文件夹并运行该文件夹中的演示程序。

### 升级SDK（可选）
如果您想要升级博世 BMV080 SDK，可以按照以下步骤操作。<br>

> [!注意]
> 由于博世目前仅提供了 BMV080 的预编译库，因此它只能支持该博世库所支持的平台。此次更新所使用的 SDK 来自博世。您可以从其官方网站下载并进行更新。[点击此处可跳转至官网](https://www.bosch-sensortec.com/products/environmental-sensors/particulate-matter-sensor/bmv080/#documents).<br>
> 本库已预存了版本为 Bosch bmv080-sdk-v11-1-0 的 SDK。您可以直接使用它，或者根据自己的需求替换为最新的 SDK。具体方法如下。

#### 安装SDK
您可以从博世官方网站下载该软件开发工具包。[点击此处可跳转至官网](https://www.bosch-sensortec.com/products/environmental-sensors/particulate-matter-sensor/bmv080/#documents).<br>

#### 替换掉原有的SDK
您需要将 DFRobot_BMV080 库中的文件替换为从下载的 SDK 中获取的文件。这一点至关重要，绝不能出错！！！<br>

##### DFRobot_BMV080 库路径
| OS | Directory|
|---|---|
|Windows | $HOME\Documents\Arduino\libraries\DFRobot_BMV080|
|Linux| $HOME/Arduino/libraries/DFRobot_BMV080|
|macOS | $HOME/Documents/Arduino/libraries/DFRobot_BMV080|
 <br>
解压已下载的 BMV080 SDK，然后按照以下路径进行替换操作。<br>

| Bosch SDK File | DFRobot_BMV080 library |
|---|---|
|api/inc/bmv080.h| src/bmv080.h|
|api/inc/bmv080_defs.h| src/bmv080_defs.h|
 <br>
以下文件可根据您的需求进行替换。只需选择相应的芯片进行更换，或者全部进行更换。<br>

| Bosch SDK File | DFRobot_BMV080 library |
|---|---|
|api/lib/xtensa_esp32/xtensa_esp32_elf_gcc/release/lib_bmv080.a | src/esp32/lib_bmv080.a|
|api/lib/xtensa_esp32/xtensa_esp32_elf_gcc/release/lib_postProcessor.a | src/esp32/lib_postProcessor.a|
|api/lib/xtensa_esp32s2/xtensa_esp32s3_elf_gcc/release/lib_postProcessor.a | src/esp32s2/lib_postProcessor.a|
|api/lib/xtensa_esp32s2/xtensa_esp32s3_elf_gcc/release/lib_bmv080.a | src/esp32s2/lib_bmv080.a|
|api/lib/xtensa_esp32s3/xtensa_esp32s3_elf_gcc/release/lib_postProcessor.a | src/esp32s3/lib_postProcessor.a|
|api/lib/xtensa_esp32s3/xtensa_esp32s3_elf_gcc/release/lib_bmv080.a | src/esp32s3/lib_bmv080.a|

## Methods

```C++
 /**
   * @fn begin
   * @brief 检查是否有 IIC 设备存在
   * @return 0 存在
   * @return -1 不存在
   */
  int begin(void);
  
  /**
   * @fn openBmv080
   * @brief 初始化BMV080传感器
   * @pre 必须先调用此函数，以便为其他函数创建所需的“句柄”
   * @post 该“句柄”必须通过“bmv080_close”函数来销毁
   * @note 它必须在与传感器进行交互的任何其他功能执行之前被调用
   * @return 如果操作成功则返回值为 0，否则则返回一个错误代码
   */
  uint8_t openBmv080(void);

  /**
   * @fn closeBmv080
   * @brief 关闭BMV080传感器
   * @pre 必须最后调用此函数，以便销毁由 _bmv080_open_ 函数创建的 _句柄_ 
   * @return 1 成功
   * @return 0 失败
   */
  bool closeBmv080(void);

  /**
   * @fn resetBmv080
   * @brief 重置一个传感器单元，该单元包括硬件和软件部分
   * @pre 必须使用由“bmv080_open”函数生成的有效“句柄”
   * @post 通过 _bmv080_set_parameter_ 更改的任何参数都会恢复到其默认值
   * @return 1 成功
   * @return 0 失败
   */
  bool resetBmv080(void);

  /**
   * @fn getBmv080DV
   * @brief 获取 BMV080 传感器的驱动程序版本
   * @param major: 主要版本号
   * @param minor: 次要版本号
   * @param patch: 补丁版本号
   * @return 1 成功
   * @return 0 失败
   */
  bool getBmv080DV(uint16_t &major, uint16_t &minor, uint16_t &patch);

  /**
   * @fn getBmv080ID
   * @brief 获取此传感器的ID号。
   * @param id: 存放ID的数组
   * @return 1 成功
   * @return 0 失败
   */
  bool getBmv080ID(char *id);

  /**
   * @fn getBmv080Data
   * @brief 获取 BMV080 传感器的数据
   * @param PM1: PM1.0 数据
   * @param PM2_5: PM2.5 数据
   * @param PM10: PM10 数据
   * @note 此功能每秒至少应被调用一次。
   * @return 1 成功, BMV080 传感器数据有效
   * @return 0 失败, BMV080 传感器数据无效
   */
  bool getBmv080Data(float *PM1, float *PM2_5, float *PM10);

  /**
   * @fn get_bmv080Data
   * @brief 将 BMV080 的数据赋值给变量 bmv080_output_t
   * @param bmv080_output: BMV080传感器数据的输出结构体
   * @note 此功能是在回调函数“getBmv080Data_cb”中被调用的
   * @return 1 成功
   * @return 0 失败
   */
  bool get_bmv080Data(bmv080_output_t bmv080_output);

  /**
   * @fn setBmv080Mode
   * @brief 设置 BMV080 传感器的模式
   * @param mode: 设置的模式可为：DFRobot_BMV080_MODE_CONTINUOUS 或 DFRobot_BMV080_MODE_DUTY_CYCLE
   *              DFRobot_BMV080_MODE_CONTINUOUS: 传感器持续进行测量
   *              DFRobot_BMV080_MODE_DUTY_CYCLE: 传感器会按照指定的时间间隔进行测量
   * @return 1 成功
   * @return 0 失败
   */
  bool setBmv080Mode(uint8_t mode);

  /**
   * @fn stopBmv080
   * @brief 停止颗粒物测量。
   * @pre 必须在数据采集周期结束时调用以确保传感器单元准备好下一个测量周期。
   * @return 1 成功
   * @return 0 错误 
   */
  bool stopBmv080(void);

  /**
   * @fn setIntegrationTime
   * @brief 测量窗口时间。
   * @note 在占空循环模式下，该测量窗口也是传感器开启时间。
   * @param integration_time 测量积分时间，单位为毫秒（ms）。
   * @return 1 成功
   * @return 0 错误
   */
  bool setIntegrationTime(float integration_time);

  /**
   * @fn getIntegrationTime
   * @brief 获取当前积分时间。
   * @return 当前积分时间，单位为毫秒（ms）。
   */
  float getIntegrationTime(void);

  /**
   * @fn setDutyCyclingPeriod
   * @brief 设置占空循环周期。
   * @n 占空循环周期（积分时间和传感器关闭/休眠时间之和）。
   * @note 此值必须比积分时间至少大2秒。
   * @param duty_cycling_period 占空循环周期，单位为毫秒（ms）。
   * @return 1 成功
   * @return 0 错误
   */
  bool setDutyCyclingPeriod(uint16_t duty_cycling_period);

  /**
   * @fn getDutyCyclingPeriod
   * @brief 获取当前占空循环周期。
   * @param duty_cycling_period 当前占空循环周期，单位为毫秒（ms）。
   * @return 1 成功
   * @return 0 错误
   */
  bool getDutyCyclingPeriod(uint16_t *duty_cycling_period);

  /**
   * @fn setObstructionDetection
   * @brief 设置是否启用阻塞检测功能。
   * @param obstructed 1 启用阻塞检测，0 禁用。
   * @return 1 成功
   * @return 0 错误
   */
  bool setObstructionDetection(bool obstructed);

  /**
   * @fn getObstructionDetection
   * @brief 获取阻塞检测功能是否启用状态。
   * @return 1 阻塞检测启用。
   * @return 0 阻塞检测禁用。
   */
  bool getObstructionDetection(void);

  /**
   * @fn setDoVibrationFiltering
   * @brief 启用或禁用振动过滤功能。
   * @param do_vibration_filtering 1 启用，0 禁用。
   * @return 1 成功
   * @return 0 错误
   */
  bool setDoVibrationFiltering(bool do_vibration_filtering);

  /**
   * @fn getDoVibrationFiltering
   * @brief 获取振动过滤功能的状态。
   * @return 1 振动过滤启用。
   * @return 0 振动过滤禁用。
   */
  bool getDoVibrationFiltering(void);

  /**
   * @fn setMeasurementAlgorithm
   * @brief 设置测量算法。
   * @param measurement_algorithm 使用的测量算法。
   *                              E_BMV080_MEASUREMENT_ALGORITHM_FAST_RESPONSE
   *                              E_BMV080_MEASUREMENT_ALGORITHM_BALANCED
   *                              E_BMV080_MEASUREMENT_ALGORITHM_HIGH_PRECISION
   * @return 1 成功
   * @return 0 错误
   */
  bool setMeasurementAlgorithm(uint8_t measurement_algorithm);

  /**
   * @fn getMeasurementAlgorithm
   * @brief 获取当前使用的测量算法。
   * @return 当前使用的测量算法。
   *         E_BMV080_MEASUREMENT_ALGORITHM_FAST_RESPONSE
   *         E_BMV080_MEASUREMENT_ALGORITHM_BALANCED
   *         E_BMV080_MEASUREMENT_ALGORITHM_HIGH_PRECISION
   */
  uint8_t getMeasurementAlgorithm(void);
```

## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Arduino uno        |              |       √      |             | 
Mega2560           |              |       √      |             | 
Leonardo           |              |       √      |             | 
ESP32              |      √       |              |             | 
micro:bit          |              |       √      |             | 
raspberry pi       |      √       |              |             |     
<br>

## History

- 2025/07/08 - Version 1.0.0  版本

## Credits

Written by Alexander(ouki.wang@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))
