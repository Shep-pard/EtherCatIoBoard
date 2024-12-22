
# Simple EtherCat IO Board

This repo contains software and hardware for a simple Ethercat IO Board to be used with linuxcnc.
It features 48 Inputs, 48 Outputs, 4 Encoder inputs (that could be used as outputs, but the softare currently doesn't support this) and 6 Analog inputs. 
The Outputs are 0,5A high side switches. And the IOs can either run on 5V or 24V (the input voltage of the board). Just change the jumper on the board. 


The project is based on:
- SOES: https://github.com/OpenEtherCATsociety/SOES
- This example implementation https://github.com/kubabuda/ecat_servo

I used the https://github.com/kubabuda/EEPROM_generator to generated my ESI files.
Thanks to those creators, you made my life very easy! 



###  License
Don't violate the original licenses. Use how you please.
No gurantees that it isn't error free. :) 
