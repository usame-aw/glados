#!/usr/bin/env python3

import sys
import serial

# Class for handling serial communications
class Serial_Talker:
    
    TERMINATOR = '\n'.encode('UTF8')

    def __init__(self, timeout=1):
        """
        Initiates Serial Connection
        """
        self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=timeout)

    def send(self, text: str):
        """
        Sends Encoded Serial Data to PICO
        """
        line = f"{text}\n"
        self.serial.write(line.encode('utf-8'))
       
    def receive(self) -> str:
        """
        Receives and Decodes Result from PICO

        return: string representing result
        """
        line = self.serial.read_until(self.TERMINATOR)
        return line.decode('utf-8').strip()

    def close(self):
        """
        Ends Serial Communications and Frees Resources
        """
        self.serial.close()






    
    
