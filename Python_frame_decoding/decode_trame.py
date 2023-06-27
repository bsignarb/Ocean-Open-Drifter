import struct
import array
import os

import numpy as np
import pandas as pd
import json

#import matplotlib.pyplot as plt
#import hvplot.pandas  # noqa
#import binascii

# Creating .txt file to store unpacked data from .sbd messages
file = "data"
file_read = file + ".txt"
IMEI_name = "300434067446590"
first_file_to_read = 287
last_file_to_read = 332
file_to_read = first_file_to_read
file_read = file + "_" + IMEI_name + "_" + str(first_file_to_read) + "-" + str(last_file_to_read) + ".txt"
isExistFile = os.path.exists(file_read)

# Erase last file if if one identical already exists
if isExistFile == True:
  os.remove(file_read)
  print ("The file has been removed")
  file_storage = open(file_read,'w')

# Scanning each files needed to be unpacked
for n in range(first_file_to_read, last_file_to_read+1):
  file_name = IMEI_name + "_000" + str(file_to_read) + ".sbd"
  file_to_read += 1
  isExist = os.path.exists(file_name)
  
  # Unpacking
  if isExist == True:
    print(file_name)
    #os.chdir("Q:\pfe_bouee\Tests_prog\Python\Messages_TAOS_juin2023")
    file = open(file_name, 'rb') 
    fileContent = file.read()
    file.close()
    frames = struct.unpack("<6fH6B6fH6B6fH6B6fH6B6fH6B6fH6B6fH6B6fH6B6fH6B6fH6B", fileContent) #320 bytes
    #print(frames)

    # Store as .txt
    file_storage = open(file_read,'a')
    with file_storage as fout:
      json.dump(frames, fout)
      file_storage.write("\n")

    # Line back at each frame and delete unnecessary characters
    f1 = open(file_read,'r+')
    input = f1.read()
    input=input.replace(' 0, 49','\n49')
    input=input.replace('[','')
    input=input.replace(']','')
    f2 = open(file_read,'w+')
    f2.write(input)
    f1.close()
    f2.close()

# Function which create dictionnary from data unpacked (with time in index)
def load_ood_data(file):
  with open(file, encoding="unicode_escape") as f:
    d = list(f.readlines())
    # replace return carriage from the end of the strings, split each string into a list
    d = [parse_line(line) for line in d]
    d = [di for di in d if di is not None]  # delete empty lines    
  df = pd.DataFrame(d).set_index("time")
  df = df.loc[ (df["longitude"]!=0) | (df["latitude"]!=0) ]  # delete lines with no correct gps location
  return df

columns = ["latitude", "longitude", "average_temperature", "temperature_RMS", "average_conductivity", "conductivity_RMS"]

# Separate columns from lines
def parse_line(line):
  items = line.split(",")
  if len(items)>1:
    time = pd.Timestamp(year=int(items[6]), month=int(items[8]), day=int(items[7]), hour=int(items[9]), minute=int(items[10]), second=int(items[11]))
    
    print(items[6])
    d = {c: float(v) for c, v in zip(columns, items[0:6])}
    return dict(time=time, **d)

df = load_ood_data(file_read)
df.head()
#df["average_temperature"].plot()

# store as .csv
df.to_csv(file_read)