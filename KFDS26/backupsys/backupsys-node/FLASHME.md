NEUTRINO SATSYS

USE TO FLASH BACKUP SYSTEM TO ESP32S3

REQUIREMENTS:
	PYTHON
	ESPTOOL.PY
	ESP32S3 VIA UART-USB BRIDGE

TO RUN:
	cd /path/to/your/project

	python -m esptool \
	 --chip esp32s3 --baud 460800 --before default_reset --after hard_reset \
	 write_flash 0x0 build/merged-binary.bin
