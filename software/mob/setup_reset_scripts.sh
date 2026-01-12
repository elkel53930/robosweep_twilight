#!/bin/bash
# ESP32 Reset Helper Script

# スクリプトを実行可能にする
chmod +x esp32_reset.py
chmod +x esp32_simple_reset.py

echo "ESP32 Reset Scripts Setup Complete!"
echo ""
echo "Usage:"
echo "1. Full featured version:"
echo "   ./esp32_reset.py -l                    # List available ports"
echo "   ./esp32_reset.py /dev/ttyUSB0          # Reset ESP32"
echo "   ./esp32_reset.py /dev/ttyUSB0 -b 9600  # Custom baud rate"
echo ""
echo "2. Simple version:"
echo "   ./esp32_simple_reset.py                # Reset on /dev/ttyUSB0"
echo "   ./esp32_simple_reset.py /dev/ttyACM0   # Reset on custom port"
echo ""
echo "Note: Install pyserial if not installed:"
echo "   pip install pyserial"