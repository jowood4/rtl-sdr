#!/usr/bin/python

import sys, time
import sqlite3 as lite
sys.path.append('../build/src/')
import rtl_power_mod

########

# Read frequency values from txt file

f = open('freq_list.txt')

string = f.readline()
freq_data = list()

while(string != ''):
    string = string.split(",")

    temp_list = list()
    for x in string:
        temp_list.append(float(x))

    freq_data.append(temp_list)

    string = f.readline()

f.close()

#Open radio
rtl_power_mod.find_and_open_dev()

#Set tuner table to settings

rtl_power_mod.initialize_tuner_values(0)
for x in freq_data:
    rtl_power_mod.set_value(freq_data.index(x), 'f', x[0])
    rtl_power_mod.set_value(freq_data.index(x), 'r', x[1])
    rtl_power_mod.set_value(freq_data.index(x), 'b', x[2])
    rtl_power_mod.set_value(freq_data.index(x), 'g', x[3])

#Initialize variables for storage
rms_pow_val = rtl_power_mod.new_doublep()
rms_pow_dc_val = rtl_power_mod.new_doublep()
db_data = list()

#Sweep radio, collect data
for x in range(0,len(freq_data)):
    temp_list = list()
    data = rtl_power_mod.new_uint8_array(pow(2,int(freq_data[x][2])))
    rtl_power_mod.set_tuner(x)
    rtl_power_mod.read_data(x, data)
    rtl_power_mod.rms_power(0, data, rms_pow_val, rms_pow_dc_val)
    rtl_power_mod.delete_uint8_array(data)
    temp_list.append(time.strftime("%d %b %Y %H:%M:%S", time.localtime()))
    temp_list.append(int(freq_data[x][0]))
    temp_list.append(int(freq_data[x][1]))
    temp_list.append(rtl_power_mod.doublep_value(rms_pow_val))
    db_data.append(temp_list)

#print rtl_power_mod.uint8_array_getitem(data, 0)
#print rtl_power_mod.doublep_value(rms_pow_val)
#print rtl_power_mod.doublep_value(rms_pow_dc_val)

#Write to database
con = None
con = lite.connect('test.db')
with con:
        cur = con.cursor()
        cur.execute("CREATE TABLE IF NOT EXISTS Data(Id INTEGER PRIMARY KEY AUTOINCREMENT, Time TEXT, Frequency INT, Bandwidth INT, Power REAL);")

        for x in db_data:
            print x
            cur.execute("INSERT INTO Data(Time, Frequency, Bandwidth, Power) VALUES(?,?,?,?);", x)

            
#Close radio
rtl_power_mod.close_dev()


