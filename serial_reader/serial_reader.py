import json

import serial


with open('hist.csv', 'w') as f:
    serial_port = serial.Serial(
        port='COM4', 
        baudrate=2000000, 
        timeout=1)
        
    with serial_port:
        first_it = True
        while True:
            j = serial_port.readline().decode('utf8')
            try:
                dic = json.loads(j)
                print(dic)

                if first_it: # titles
                    f.write(','.join([str(it) for it in dic])+'\n')
                    first_it = False
                
                f.write(','.join([str(it) for it in dic.values()])+'\n')
            except json.decoder.JSONDecodeError:
                print(f'Not JSON: "{j}"')
            