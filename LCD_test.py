import Jetson.GPIO as GPIO
import time

# Pin setup (adjust if using different GPIO pins)
LCD_RS = 22
LCD_E  = 27
LCD_D4 = 17
LCD_D5 = 18
LCD_D6 = 23
LCD_D7 = 24
LCD_WIDTH = 16   # Characters per line
LCD_LINE_1 = 0x80  # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0  # LCD RAM address for the 1st line
E_PULSE = 0.0005
E_DELAY = 0.0005

def lcd_init():
    lcd_byte(0x33, False)  # Initialize
    lcd_byte(0x32, False)  # 4-bit mode
    lcd_byte(0x28, False)  # 2 line, 5x7 matrix
    lcd_byte(0x0C, False)  # Display on, cursor off
    lcd_byte(0x06, False)  # Increment cursor
    lcd_byte(0x01, False)  # Clear display
    time.sleep(E_DELAY)

def lcd_byte(bits, mode):
    GPIO.output(LCD_RS, mode)
    # High bits
    GPIO.output(LCD_D4, bool(bits & 0x10))
    GPIO.output(LCD_D5, bool(bits & 0x20))
    GPIO.output(LCD_D6, bool(bits & 0x40))
    GPIO.output(LCD_D7, bool(bits & 0x80))
    lcd_toggle_enable()

    # Low bits
    GPIO.output(LCD_D4, bool(bits & 0x01))
    GPIO.output(LCD_D5, bool(bits & 0x02))
    GPIO.output(LCD_D6, bool(bits & 0x04))
    GPIO.output(LCD_D7, bool(bits & 0x08))
    lcd_toggle_enable()

def lcd_toggle_enable():
    time.sleep(E_DELAY)
    GPIO.output(LCD_E, True)
    time.sleep(E_PULSE)
    GPIO.output(LCD_E, False)
    time.sleep(E_DELAY)

def lcd_string(message, line):
    message = message.ljust(LCD_WIDTH, " ")
    lcd_byte(line, False)
    for char in message:
        lcd_byte(ord(char), True)

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup([LCD_E, LCD_RS, LCD_D4, LCD_D5, LCD_D6, LCD_D7], GPIO.OUT)
    lcd_init()

    try:
        # Display "HELLO!" on the first line
        lcd_string("HELLO", LCD_LINE_1)
        lcd_string("WORLD", LCD_LINE_2)
        while True:
            time.sleep(1)  # Keep displaying the text
    except KeyboardInterrupt:
        pass
    finally:
        lcd_byte(0x01, False)  # Clear display before exiting
        GPIO.cleanup()

if __name__ == '__main__':
    main()
