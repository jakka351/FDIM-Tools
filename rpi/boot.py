# boot.py — runs once at power-up before code.py (CircuitPython).
# Enables the second USB serial port (usb_cdc.data) that code.py uses for the BTN/REL/KEY
# event stream and the mapping tool, and names the device so it's easy to find on the Pi:
#   /dev/serial/by-id/usb-TesterPresent_FG-ICC-Buttons-*-if02   (data channel)
#   /dev/input/by-id/usb-TesterPresent_FG-ICC-Buttons-*-event-kbd (HID keyboard)
import usb_cdc
import supervisor

usb_cdc.enable(console=True, data=True)     # console = REPL/debug, data = event stream
try:
    supervisor.set_usb_identification(manufacturer="TesterPresent", product="FG-ICC-Buttons")
except Exception:
    pass                                     # older CircuitPython: default names, still fine
