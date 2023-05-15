
'''
// Decoder function from the UART data buffer. 
void decode_uart_data(uint8_t *uart_rx_buffer) {
  // Check the first byte of the buffer
  switch (uart_rx_buffer[0]) {
    case 0x01: // Drive
      // Check the second byte of the buffer
      switch (uart_rx_buffer[1]) {
        case 0x01: // Forward
          // Enable the drive relay
          enable_drive();
          // Enable the forward relay
          enable_forward();
          // Run the drive motor forward
          drive_forward();
          break;
        case 0x02: // Reverse
          // Enable the drive relay
          enable_drive();
          // Enable the reverse relay
          enable_reverse();
          // Run the drive motor backward
          drive_backward();
          break;
        case 0x03: // Stop
          // Disable the drive relay
          disable_drive();
          // Stop the drive motor
          drive_stop();
          break;
      }
      break;
    case 0x02: // Steering
      // Check the second byte of the buffer
      switch (uart_rx_buffer[1]) {
        case 0x01: // Forward
          // Run the steering motor forward
          steer_forward();
          break;
        case 0x02: // Reverse
          // Run the steering motor backward
          steer_backward();
          break;
        case 0x03: // Stop
          // Stop the steering motor
          steer_stop();
          break;
      }
      break;
    case 0x03: // Brake
      // Check the second byte of the buffer
      switch (uart_rx_buffer[1]) {
        case 0x01: // Forward
          // Run the brake motor forward
          brake_forward();
          break;
        case 0x02: // Reverse
          // Run the brake motor backward
          brake_backward();
          break;
        case 0x03: // Stop
          // Stop the brake motor
          brake_stop();
          break;
      }
      break;
    case 0x04: // Speed with 2 preset speeds
      // Check the second byte of the buffer
      switch (uart_rx_buffer[1]) {
        case 0x01: // Low
          // Set the speed to low
          set_speed(0x00);
          break;
        case 0x02: // High
          // Set the speed to high
          set_speed(128);
          break;
        case 0x03: // Increment
          // Increment the speed
          increment_speed();
          break;
        case 0x04: // Decrement
          // Decrement the speed
          decrement_speed();
          break;
        case 0x05: // Stop
          // Stop the drive motor
          set_speed(0x00);
          disable_drive();
          break;
      }
      break;
  }
}
'''

# Looking at the decoder function from the above code, we can map the uart values to a controller button
# Button mapping:
#   Steer Right: Right Bumper
#   Steer Left: Left Bumper
#   Drive Forward: DPAD Up
#   Drive Backward: DPAD Down
#   Brake: A
#   Release Brake: B
#   Stop: Start
#   Release Stop: Select
#   Speed Up: Right Trigger
#   Speed Down: Left Trigger
#   Set Speed to Low: X
#   Set Speed to High: Y

# Library Imports
import time
import serial
import threading
import game_controller

# Global Variables
controller = game_controller.Controller()
uart = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

# Main Function
def main():
    # Start the controller listener
    controller.startListener()

    # Main loop
    while True:

        # Check the controller buttons
        if controller.buttons['BTN_SOUTH']:
            # Send the brake command
            uart.write(b'\x03\x01')

        elif controller.buttons['BTN_EAST']:
            # Send the release brake command
            uart.write(b'\x03\x02')

        elif controller.buttons['BTN_START']:
            # Send the stop command
            uart.write(b'\x03\x03')

        elif controller.buttons['BTN_SELECT']:
            # Send the release stop command
            uart.write(b'\x03\x04')

        elif controller.buttons['BTN_WEST']:
            # Send the speed low command
            uart.write(b'\x04\x01')

        elif controller.buttons['BTN_NORTH']:
            # Send the speed high command
            uart.write(b'\x04\x02')

        elif controller.buttons['BTN_TL']:
            # Send the speed down command
            uart.write(b'\x04\x04')

        elif controller.buttons['BTN_TR']:
            # Send the speed up command
            uart.write(b'\x04\x03')

        elif controller.buttons['BTN_TL2']:
            # Send the steer left command
            uart.write(b'\x02\x01')

        elif controller.buttons['BTN_TR2']:
            # Send the steer right command
            uart.write(b'\x02\x02')

        elif controller.buttons['DPAD_SOUTH']:
            # Send the drive backward command
            uart.write(b'\x01\x02')

        elif controller.buttons['DPAD_NORTH']:
            # Send the drive forward command
            uart.write(b'\x01\x01')

        elif controller.buttons['DPAD_EAST']:
            # Send the drive stop command
            uart.write(b'\x01\x03')

        elif controller.buttons['DPAD_WEST']:
            # Send the drive stop command
            uart.write(b'\x01\x03')

        # Sleep for 0.1 seconds
        time.sleep(0.1)