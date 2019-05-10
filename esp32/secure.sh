python /home/regis/repos/pycom-esp-idf/components/esptool_py/esptool/espefuse.py --do-not-confirm --port /dev/ttyUSB0 burn_key flash_encryption flash_encryption_key.bin
python /home/regis/repos/pycom-esp-idf/components/esptool_py/esptool/espefuse.py --do-not-confirm --port /dev/ttyUSB0 burn_key secure_boot secure-bootloader-key.bin
python /home/regis/repos/pycom-esp-idf/components/esptool_py/esptool/espefuse.py --port /dev/ttyUSB0 burn_efuse FLASH_CRYPT_CNT
python /home/regis/repos/pycom-esp-idf/components/esptool_py/esptool/espefuse.py --port /dev/ttyUSB0 burn_efuse FLASH_CRYPT_CONFIG 0x0F
python /home/regis/repos/pycom-esp-idf/components/esptool_py/esptool/espefuse.py --port /dev/ttyUSB0 burn_efuse ABS_DONE_0

