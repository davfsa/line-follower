# Infrared Slotted Optical Sensor

More information on the [Amazon page](<https://www.amazon.es/gp/product/B0B6SPB2C4/ref=ppx_yo_dt_b_asin_title_o04_s00>)

## Description
The slot type photoelectric sensor is composed of an infrared light emitting diode and an NPN phototransistor
with a groove width of 5.9 mm. As long as the non-transparent object passes through the slot type, it can be
triggered (used with the shop's car code wheel) to output TTL low level. The Schmitt trigger is used to shake
the pulse, which is very stable and can be used for measuring the speed of the car, measuring the distance and
the like! M3 screw mounting holes at both ends.

## Specifications

**Working voltage**: 3.3V-5V

**Output form**: digital switch OUT output (0 and 1)

When VCC and GND are connected, the module signal indicator will be on. When there is no block in the module
slot, the receiving tube is turned on, and the module OUT outputs high level. When blocking, the OUT output is
low and the signal indicator is off. The module OUT can be connected to the relay to form a limit switch and
other functions, and can also be connected to the active buzzer module to form an alarm. The OUT output
interface can be directly connected to the IO port of the MCU. Generally, it is connected to an external
interrupt to detect whether the sensor has a occlusion. For example, the motor code wheel can detect the motor
speed.
