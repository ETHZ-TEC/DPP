
#include "sensor.h"


/**
 * Sensor I2C interface configuration (with clock source frequency placeholder)
 */
eUSCI_I2C_MasterConfig sensor_i2cConfig = {
  .selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK,
  .i2cClk = 0,
  .dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS,
  .byteCounterThreshold = 0,
  .autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP,
};


/**
 * Initialize and enable the low-level peripherals used for the sensors.
 */
void sensor_init(void)
{
  // initialize GPIO first
  GPIO_setAsPeripheralModuleFunctionInputPin(SENSOR_I2C_SCL_PORT, SENSOR_I2C_SCL_PIN, SENSOR_I2C_GPIO_AF);
  GPIO_setAsPeripheralModuleFunctionInputPin(SENSOR_I2C_SDA_PORT, SENSOR_I2C_SDA_PIN, SENSOR_I2C_GPIO_AF);

  // I2C master configuration
  sensor_i2cConfig.i2cClk = CS_getSMCLK();
  I2C_initMaster(SENSOR_I2C_INTERFACE, &sensor_i2cConfig);

  // enable interface
  I2C_enableModule(SENSOR_I2C_INTERFACE);
}


/**
 * Read data from one or multiple sensor register(s).
 * @param device_address The sensors I2C bus address
 * @param register_address The sensors register address to start reading from
 * @param data Pointer to the memory where the received data is stored
 * @param count Number of bytes to read
 * @return Status code, zero on success, non-zero in case of errors
 */
int8_t sensor_i2c_read(uint8_t device_address, uint8_t register_address,
             uint8_t* data, uint16_t count)
{
  /*
   * Data on the bus should be like
   * |------------+---------------------|
   * | I2C action | Data                |
   * |------------+---------------------|
   * | Start      | [device_address]    |
   * | Write      | (register_address)  |
   * | Start      | [device_address]    |
   * | Read       | (data[0])           |
   * | Read       | (....)              |
   * | Read       | (data[count - 1])   |
   * | Stop       | -                   |
   * |------------+---------------------|
   *
   * return 0 for success, non-zero for failure
   */

  // wait for I2C to be ready to read
  while (I2C_isBusBusy(SENSOR_I2C_INTERFACE)) {
    continue;
  }

  // configure slave address
  I2C_setSlaveAddress(SENSOR_I2C_INTERFACE, device_address);

  // initiate read with START and writing register address
  I2C_masterSendMultiByteStart(SENSOR_I2C_INTERFACE, register_address);

  // wait for transmission to complete, clear interrupt flag
  while(!((I2C_MODULE)->IFG & UCTXIFG0));
  /*while (!I2C_getInterruptStatus(SENSOR_I2C_INTERFACE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0)) {
    continue;
  }*/
  I2C_clearInterruptFlag(SENSOR_I2C_INTERFACE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);

  // check for NACK, on NACK stop and exit with failure
  if (I2C_getInterruptStatus(SENSOR_I2C_INTERFACE, EUSCI_B_I2C_NAK_INTERRUPT)) {
    I2C_masterSendMultiByteStop(SENSOR_I2C_INTERFACE);
    return -1;
  }

  // start receive transaction with RE-START
  I2C_masterReceiveStart(SENSOR_I2C_INTERFACE);

  // wait for RE-START to complete send
  while(I2C_masterIsStartSent(SENSOR_I2C_INTERFACE)) {
    continue;
  }

  // receive all but last data byte
  while (count > 1) {
    // wait for next byte transmission to complete
    while (!I2C_getInterruptStatus(SENSOR_I2C_INTERFACE, EUSCI_B_I2C_RECEIVE_INTERRUPT0)) {
      continue;
    }

    // read received byte, clears RX interrupt automatically
    *data = I2C_masterReceiveMultiByteNext(SENSOR_I2C_INTERFACE);
    data = data + 1;

    count = count - 1;
  }

  // --- broken driverlib implementation to handle reception of last byte and STOPing ---
  // *data = I2C_masterReceiveMultiByteFinish(SENSOR_I2C_INTERFACE);
  // return 0;
  // --- alternative handling using driverlib API below ---

  // receive last byte and generate stop condition
  I2C_masterReceiveMultiByteStop(SENSOR_I2C_INTERFACE);

  // wait for stop interrupt
  while (!I2C_getInterruptStatus(SENSOR_I2C_INTERFACE, EUSCI_B_I2C_STOP_INTERRUPT)) {
    continue;
  }

  // wait for next byte transmission to complete
  while (!I2C_getInterruptStatus(SENSOR_I2C_INTERFACE, EUSCI_B_I2C_RECEIVE_INTERRUPT0)) {
    continue;
  }

  // actually read last reveiced value
  *data = I2C_masterReceiveMultiByteNext(SENSOR_I2C_INTERFACE);
  return 0;
}

/**
 * Write data to one or multiple sensor register(s).
 * @param device_address The sensors I2C bus address
 * @param register_address The sensors register address to write
 * @param data Pointer to the data to be written
 * @param count Number of bytes to write
 * @return Status code, zero on success, non-zero in case of errors
 */
int8_t sensor_i2c_write(uint8_t device_address, uint8_t register_address,
            uint8_t *data, uint16_t count)
{
  /*
   * Data on the bus should be like
   * |------------+---------------------|
   * | I2C action | Data                |
   * |------------+---------------------|
   * | Start      | [device_address]    |
   * | Write      | (register_address)  |
   * | Write      | (data[0])           |
   * | Write      | (...)               |
   * | Write      | (data[count - 1])   |
   * | Stop       | -                   |
   * |------------+---------------------|
   *
   * return 0 for success, non-zero for failure
   */

  // wait for I2C to be ready to write
  while (I2C_isBusBusy(SENSOR_I2C_INTERFACE)) {
    continue;
  }

  // configure slave address
  I2C_setSlaveAddress(SENSOR_I2C_INTERFACE, device_address);

  // initiate read with START and writing register address
  I2C_masterSendMultiByteStart(SENSOR_I2C_INTERFACE, register_address);

  // wait for transmission to complete
  while (!I2C_getInterruptStatus(SENSOR_I2C_INTERFACE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0)) {
    continue;
  }

  // check for NACK, on NACK stop and exit with failure
  if (I2C_getInterruptStatus(SENSOR_I2C_INTERFACE, EUSCI_B_I2C_NAK_INTERRUPT)) {
    I2C_masterSendMultiByteStop(SENSOR_I2C_INTERFACE);
    return -1;
  }

  // send data
  while (count > 0) {
    // wait for next byte transmission to complete
    while (!I2C_getInterruptStatus(SENSOR_I2C_INTERFACE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0)) {
      continue;
    }

    // send next byte, clears TX interrupt automatically
    I2C_masterSendMultiByteNext(SENSOR_I2C_INTERFACE, *data);
    data = data + 1;

    count = count - 1;
  }

  // wait for last transmission to complete, send STOP and finally clear flag
  while (!I2C_getInterruptStatus(SENSOR_I2C_INTERFACE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0)) {
    continue;
  }
  I2C_masterSendMultiByteStop(SENSOR_I2C_INTERFACE);
  I2C_clearInterruptFlag(SENSOR_I2C_INTERFACE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);

  return 0;
}

/**
 * Blocking delay for sensor initialization
 * @param period Delay in ms for which the task is blocked
 */
void sensor_delay_ms(uint32_t period)
{
  while (period)
  {
    __delay_cycles(MCLK_SPEED / 1000);
    period--;
  }
}
