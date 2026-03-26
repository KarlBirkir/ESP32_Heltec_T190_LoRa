PlatformIO project
Intended for two of the Heltec T190. Great for testing node positions, antennas, modulation settings, etc.
A message is sent back and forth, reporting on RSSI and SNR both ways. Keeps a little history over this, including the LoRa settings used
The GUI lets you change the LoRa modulation settings, SF, BW and Freq.  After changing, a change request is sent to the other node and change takes place after your node receives ACK.
