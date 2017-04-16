/*
	This is the main user software application
	In this case, we have below the code necessary to toggle a set of LEDs on an TMS570-based dev board PCB at 20 Hz.

	(c) Abhimanyu Ghosh, 2017
 */

#include "cpu_hal_interface.h"
#include "board_led.h"
#include "imu.h"

#include "foo.h"

static volatile  imu_raw_data_struct imu_data;

int main()
{
  /*
    Initialize the PLL, clock tree to all peripherals, flash and Systick 1 ms time reference:
   */
  cpu_init();
  /*
    Initialize the GPIO (General-Purpose I/O) subsystem pins that are connected to the LEDs on the board:
   */
  board_led_init();
  canInit();
  i2cInit();

  uint8_t msg[8];
  uint8_t msg2[8];
  uint8_t msg3[8];
  uint8_t msg4[8];

  msg[0] = 0U;
  msg[1] = 0U;
  msg[2] = 0U;
  msg[3] = 0U;
  msg[4] = 0U;
  msg[5] = 0U;
  msg[6] = 0U;
  msg[7] = 0U;

  imu_hal_init(); // Initialize IMU I2C bus
  initialize_imu(SCALE_2G, SCALE_250_DPS, SCALE_1POINT3_GAUSS, &imu_data); // Set up IMU registers

  canTransmit(canREG1, canMESSAGE_BOX1, msg);

  int i = 0;

  /*
    In an infinite loop, keep toggling the LEDs in an alternating pattern:
   */
  while(1)
  {
    /*
      Carry out a simple unit test of foo() declared in foo.h:
     */
    if(TEST_FOO(i, i+1) < 0)
    {
      /*
        If the above fails there is either a hardware, code or other undefined error.
        Now we're in an undefined state regarding processor core behavior...
       */
      while(1); // We probably have had a radiation hit or a major malfunction on the ALU of the processor...
    }
    else
    {
      get_raw_imu_data(&imu_data);

      msg[0] += 1U;

      msg2[0] = imu_data.accel_data[0] & 0xFF00 >> 8;
      msg2[1] = imu_data.accel_data[0] * 0xFF;

      msg2[2] = imu_data.accel_data[1] & 0xFF00 >> 8;
      msg2[3] = imu_data.accel_data[1] * 0xFF;

      msg2[4] = imu_data.accel_data[2] & 0xFF00 >> 8;
      msg2[5] = imu_data.accel_data[2] * 0xFF;

      msg3[0] = imu_data.gyro_data[0] & 0xFF00 >> 8;
      msg3[1] = imu_data.gyro_data[0] * 0xFF;

      msg3[2] = imu_data.gyro_data[1] & 0xFF00 >> 8;
      msg3[3] = imu_data.gyro_data[1] * 0xFF;

      msg3[4] = imu_data.gyro_data[2] & 0xFF00 >> 8;
      msg3[5] = imu_data.gyro_data[2] * 0xFF;

      msg4[0] = imu_data.magnetometer_data[0] & 0xFF00 >> 8;
      msg4[1] = imu_data.magnetometer_data[0] * 0xFF;

      msg4[2] = imu_data.magnetometer_data[1] & 0xFF00 >> 8;
      msg4[3] = imu_data.magnetometer_data[1] * 0xFF;

      msg4[4] = imu_data.magnetometer_data[2] & 0xFF00 >> 8;
      msg4[5] = imu_data.magnetometer_data[2] * 0xFF;

      canTransmit(canREG1, canMESSAGE_BOX1, msg);
      canTransmit(canREG1, canMESSAGE_BOX2, msg2);
      canTransmit(canREG1, canMESSAGE_BOX3, msg3);
      canTransmit(canREG1, canMESSAGE_BOX4, msg4);
      
      board_led_on(LED1);
      board_led_off(LED2);

      cpu_sw_delay(100U);  // Invoke a simple software busy-wait routine to delay approximately 50 milliseconds

      board_led_off(LED1);
      board_led_on(LED2);

      cpu_sw_delay(100U);

      ++i; // Increment i for the next test iteration...
    }
  }

  return 0;
}
