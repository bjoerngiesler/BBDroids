import sys
import serial
import threading
import imgui
from imgui.integrations.qt import QtGl

class SerialTerminal:
    def __init__(self):
        self.serial_port = None
        self.output_buffer = ""
        self.input_buffer = ""
        self.is_running = False

    def open_port(self, port, baudrate):
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.is_running = True
        threading.Thread(target=self.read_from_port, daemon=True).start()

    def read_from_port(self):
        while self.is_running:
            if self.serial_port and self.serial_port.is_open:
                line = self.serial_port.readline().decode('utf-8', errors='ignore')
                if line:
                    self.output_buffer += line

    def write_to_port(self, data):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(data.encode('utf-8'))

    def close_port(self):
        self.is_running = False
        if self.serial_port is not None:
            self.serial_port.close()

    def get_output(self):
        return self.output_buffer

def main():
    terminal = SerialTerminal()

    app = QtGl.QtApplication(sys.argv)

    while not app.should_stop:
        imgui.new_frame()

        imgui.begin("Serial Terminal")
        
        imgui.text("Output:")
        imgui.begin_child("output", 400, 300, True)
        imgui.text(terminal.get_output())
        imgui.end_child()

        imgui.input_text("Input", terminal.input_buffer, 256)
        if imgui.button("Send"):
            terminal.write_to_port(terminal.input_buffer)
            terminal.input_buffer = ""

        imgui.end()

        imgui.render()
        app.render()

    terminal.close_port()
    sys.exit()

if __name__ == "__main__":
    main()
