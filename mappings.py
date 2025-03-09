import Jetson.GPIO as GPIO

model, jetson_info, channel_data = GPIO.gpio_pin_data.get_data()

# print(type(channel_data['BCM']))
for pin in channel_data['BCM'].values():
    print(pin.channel, " for " ,pin.gpio_name)

print("-----------------------")
for pin in channel_data['BOARD'].values():
    print(pin.channel, " for " ,pin.gpio_name)

