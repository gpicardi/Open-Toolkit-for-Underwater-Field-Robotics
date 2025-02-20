# Distributed with a free-will license.
# Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
# MS8607_02BA
# This code is designed to work with the MS8607_02BA_I2CS I2C Mini Module available from ControlEverything.com.
# https://www.controleverything.com/products

import smbus
import time

# Get I2C bus
bus = smbus.SMBus(0)

# MS8607_02BA address, 0x76(118)
#		0x1E(30)	Reset command
bus.write_byte(0x76, 0x1E)

time.sleep(0.5)

# Read 12 bytes of calibration data
# Pressure sensitivity | SENST1
data1 = bus.read_i2c_block_data(0x76, 0xA2, 2)
# Pressure offset | OFFT1
data2 = bus.read_i2c_block_data(0x76, 0xA4, 2)
# Temperature coefficient of pressure sensitivity | TCS
data3 = bus.read_i2c_block_data(0x76, 0xA6, 2)
# Temperature coefficient of pressure offset | TCO
data4 = bus.read_i2c_block_data(0x76, 0xA8, 2)
# Reference temperature | TREF
data5 = bus.read_i2c_block_data(0x76, 0xAA, 2)
# Temperature coefficient of the temperature | TEMPSENS
data6 = bus.read_i2c_block_data(0x76, 0xAC, 2)

# Convert the data
c1 = data1[0] * 256 + data1[1]
c2 = data2[0] * 256 + data2[1]
c3 = data3[0] * 256 + data3[1]
c4 = data4[0] * 256 + data4[1]
c5 = data5[0] * 256 + data5[1]
c6 = data6[0] * 256 + data6[1]

# MS8607_02BA address, 0x76(118)
#		0x40(64)	Initiate pressure conversion(OSR = 256)
bus.write_byte(0x76, 0x40)

time.sleep(0.5)

# Read data back from 0x00(0), 3 bytes, D1 MSB2, D1 MSB1, D1 LSB
# Digital pressure value
data = bus.read_i2c_block_data(0x76, 0x00, 3)

D1 = data[0] * 65536 + data[1] * 256 + data[2]

# MS8607_02BA address, 0x76(118)
#		0x50(64)	Initiate temperature conversion(OSR = 256)
bus.write_byte(0x76, 0x50)

time.sleep(0.5)

# Read data back from 0x00(0), 3 bytes, D2 MSB2, D2 MSB1, D2 LSB
# Digital temperature value
data0 = bus.read_i2c_block_data(0x76, 0x00, 3)

# Convert the data
D2 = data0[0] * 65536 + data0[1] * 256 + data0[2]
dT = D2 - c5 * 256
Temp = 2000 + dT * c6 / 8388608
OFF = c2 * 131072 + (c4 * dT) / 64
SENS = c1 * 65536 + (c3 * dT ) / 128

if Temp >= 2000 :
	Ti = 5 * (dT * dT) / 274877906944
	OFFi = 0
	SENSi= 0
elif Temp < 2000 :
	Ti = 3 * (dT * dT) / 8589934592
	OFFi= 61 * ((Temp - 2000) * (Temp - 2000)) / 16
	SENSi= 29 * ((Temp - 2000) * (Temp - 2000)) / 16
	if Temp < -1500:
		OFFi = OFFi + 17 * ((Temp + 1500) * (Temp + 1500))
		SENSi = SENSi + 9 * ((Temp + 1500) * (Temp +1500))
OFF2 = OFF - OFFi
SENS2= SENS - SENSi
cTemp = (Temp - Ti) / 100.0
fTemp =  cTemp * 1.8 + 32
pressure = ((((D1 * SENS2) / 2097152) - OFF2) / 32768.0) / 100.0

# MS8607_02BA address, 0x40(64)
#		0xFE(254)	Send reset command
bus.write_byte(0x40, 0xFE)

time.sleep(0.3)

# MS8607_02BA address, 0x40(64)
#		0xF5(245)	Send NO Hold master command
bus.write_byte(0x40, 0xF5)

time.sleep(0.5)

# MS8607_02BA address, 0x40(64)
# Read data back from device
data0 = bus.read_byte(0x40)
data1 = 0

# Convert the data
D3 = data0 * 256 + data1

humidity = (-6.0 + (125.0 * (D3 / 65536.0)))

# Output data to screen
print("Relative Humidity : %.2f %%" %humidity)
print("Temperature compensated Pressure is : %.2f mbar" %pressure)
print ("Temperature in Celsius : %.2f C" %cTemp)
print ("Temperature in Fahrenheit : %.2f F" %fTemp)