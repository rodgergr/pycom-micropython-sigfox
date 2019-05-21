from machine import UART
import os
import micropython
uart = UART(0, 115200)
os.dupterm(uart)

micropython.alloc_emergency_exception_buf(100)
