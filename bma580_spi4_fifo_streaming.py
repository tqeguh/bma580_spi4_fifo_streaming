#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BMA580 FIFO acquisition over 4-wire SPI (Shuttle 580 + App Board 3.1).

Adapted from Bosch Sensortec's BMA400 interrupt streaming example.
Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
BSD-3-Clause

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

The BMA580's INT1/INT2 pads become SDO/CSB in SPI 4-wire mode. This version
reads full 16-bit XYZ FIFO frames with the sensor's 24-bit timestamp appended
to every frame, as in Bosch's fifo_full_16_bit_each_frame_comp_dis.c example.
The timestamp runs at 3200 ticks/s. It is not synchronized to an analog DAQ.
"""

import argparse
import csv
import struct
import time
from pathlib import Path

import coinespy as cpy
from coinespy import ErrorCodes


BUS = cpy.SPIBus.BUS_SPI_0
CS = cpy.MultiIOPin.MINI_SHUTTLE_PIN_2_1  # App 3.1 shuttle P2.1 CS
SPI_DUMMY_BYTES = 1
DATA_BYTES = 6
GRAVITY = 9.80665
SENSOR_TICK_SECONDS = 0.0003125
SENSOR_TIME_MODULUS = 1 << 24
FIFO_FRAME_BYTES = 10  # 0xCF header + 6 accel data + 3 sensor time
FIFO_PAYLOAD_BYTES = 9  # FIFO_LEVEL excludes the header
CHIP_ID = 0x00
HEALTH = 0x02
CMD_SUSPEND = 0x04
SENSOR_STATUS = 0x11
ACC_DATA = 0x18
FIFO_LEVEL = 0x22
FIFO_DATA = 0x24
ACC_CONF_0 = 0x30
ACC_CONF_1 = 0x31
ACC_CONF_2 = 0x32
IF_CONF_1 = 0x3B
FIFO_CONF_0 = 0x41
FIFO_CONF_1 = 0x42
ODR_CODES = {100: 0x06, 200: 0x07, 400: 0x08, 800: 0x09,
             1600: 0x0A, 3200: 0x0B, 6400: 0x0C}


def check(result, operation):
    if result != ErrorCodes.COINES_SUCCESS:
        raise RuntimeError(f"{operation}: COINES returned {result}")


class BMA580:
    def __init__(self):
        self.board = cpy.CoinesBoard()
        self.opened = False
        self.powered = False

    def open(self):
        check(self.board.open_comm_interface(cpy.CommInterface.USB), "open board")
        self.opened = True
        info = self.board.get_board_info()
        if not info.HardwareId:
            raise RuntimeError("Application Board not detected")
        print(f"Board: {hex(info.Board)}, shuttle ID: {hex(info.ShuttleID)}")
        check(self.board.set_shuttleboard_vdd_vddio_config(vdd_val=0, vddio_val=0),
              "switch sensor power off")
        check(self.board.config_spi_bus(BUS, CS, cpy.SPISpeed.SPI_10_MHZ,
                                        cpy.SPIMode.MODE0), "configure SPI")
        check(self.board.set_pin_config(CS, cpy.PinDirection.OUTPUT,
                                        cpy.PinValue.HIGH), "configure CS")
        check(self.board.set_shuttleboard_vdd_vddio_config(vdd_val=3.3,
                                                            vddio_val=3.3), "power sensor")
        self.powered = True
        time.sleep(0.01)

    def read(self, reg, length=1):
        # COINES read_spi already inserts the R/W address bit. The BMA580
        # returns one SPI dummy byte BEFORE the requested register bytes.
        data = self.board.read_spi(BUS, reg, length + SPI_DUMMY_BYTES)
        check(self.board.error_code, f"SPI read 0x{reg:02X}")
        if len(data) != length + SPI_DUMMY_BYTES:
            raise IOError(f"SPI read 0x{reg:02X}: got {len(data)} bytes")
        return bytes(data[SPI_DUMMY_BYTES:])

    def read8(self, reg):
        return self.read(reg)[0]

    def write(self, reg, value, mask=0xFF):
        check(self.board.write_spi(BUS, reg, value), f"SPI write 0x{reg:02X}")
        time.sleep(0.001)  # Sensor requires >=2 us idle after ACTIVE write.
        actual = self.read8(reg)
        if actual & mask != value & mask:
            raise IOError(f"Register 0x{reg:02X}: wrote {value:#04x}, read {actual:#04x}")

    def configure_sensor(self, odr_hz, range_g):
        self.read(CHIP_ID)  # First SPI transaction selects SPI 4-wire.
        chip = self.read8(CHIP_ID)
        if chip != 0xC4:
            raise IOError(f"Wrong chip ID: 0x{chip:02X}, expected 0xC4")
        self.write(CMD_SUSPEND, 0x00)
        time.sleep(0.01)
        if self.read8(HEALTH) & 0x0F != 0x0F:
            raise IOError("BMA580 health status did not become 0x0F")
        if self.read8(IF_CONF_1) & 0x02:
            raise IOError("Sensor already in 3-wire SPI mode; power-cycle it")

        self.write(ACC_CONF_0, 0x00, mask=0x0F)  # Disable while configuring.
        self.write(ACC_CONF_1, 0xA0 | ODR_CODES[odr_hz])  # HPM, NORM_AVG4.
        range_code = {2: 0, 4: 1, 8: 2, 16: 3}[range_g]
        self.write(ACC_CONF_2, 0x08 | range_code, mask=0x1F)  # IIR -40 dB, low noise.
        # 512-byte FIFO, stop on full, append sensor time to every frame.
        self.write(FIFO_CONF_0, 0x00, mask=0x1F)
        self.write(FIFO_CONF_1, 0x1A, mask=0x1F)
        self.write(FIFO_CONF_0, 0x0F, mask=0x1F)  # XYZ, 16-bit, FIFO on.
        self.write(ACC_CONF_0, 0x0F, mask=0x0F)
        time.sleep(0.02)
        print(f"BMA580 ID 0x{chip:02X}; +/-{range_g} g; ODR {odr_hz} Hz")

    def fifo_level(self):
        low, high = self.read(FIFO_LEVEL, 2)
        return low | ((high & 0x07) << 8)

    def preflight(self, range_g):
        """Fail before writing CSV if direct register values are all zero."""
        snapshots = []
        for _ in range(3):
            snapshots.append(self.read(ACC_DATA, DATA_BYTES))
            time.sleep(0.005)
        xyz = struct.unpack("<hhh", snapshots[-1])
        print("Direct ACC_DATA bytes:", snapshots[-1].hex(" "))
        print("Direct XYZ [g]:", tuple(round(a * range_g / 32768, 5) for a in xyz))
        if all(raw == bytes(DATA_BYTES) for raw in snapshots):
            status = self.read8(SENSOR_STATUS)
            raise RuntimeError(
                f"Direct ACC_DATA is zero before FIFO capture (SENSOR_STATUS=0x{status:02X}). "
                "Check sensor setup and SPI readout; no zero CSV was written."
            )

    def read_fifo_frames(self, frame_count):
        return self.read(FIFO_DATA, FIFO_FRAME_BYTES * frame_count)

    def close(self):
        if self.opened:
            try:
                if self.powered:
                    self.board.set_shuttleboard_vdd_vddio_config(vdd_val=0,
                                                                  vddio_val=0)
            finally:
                self.board.close_comm_interface()
                self.opened = False


def decode_fifo_frame(frame, range_g):
    if len(frame) != FIFO_FRAME_BYTES or frame[0] != 0xCF:
        raise ValueError(f"Unexpected FIFO header/frame: {bytes(frame).hex(' ')}")
    raw = struct.unpack("<hhh", bytes(frame[1:7]))
    tick = int.from_bytes(frame[7:10], "little")
    xyz_g = tuple(value * range_g / 32768.0 for value in raw)
    return tick, raw, xyz_g


def capture(samples, odr_hz, range_g, output, burst_frames=40):
    if samples < 1:
        raise ValueError("samples must be positive")
    if odr_hz not in ODR_CODES:
        raise ValueError(f"ODR must be one of {sorted(ODR_CODES)} Hz")
    if range_g not in (2, 4, 8, 16):
        raise ValueError("range must be 2, 4, 8, or 16 g")
    if not 1 <= burst_frames <= 45:
        raise ValueError("burst_frames must be 1 to 45 (fits 512-byte FIFO)")

    sensor = BMA580()
    count = 0
    gaps = 0
    first_tick = None
    previous_tick = None
    unwrapped_ticks = 0
    try:
        sensor.open()
        sensor.configure_sensor(odr_hz, range_g)
        sensor.preflight(range_g)
        with Path(output).open("w", newline="", encoding="utf-8") as out:
            writer = csv.writer(out)
            writer.writerow(("sample", "sensor_time_ticks_24bit", "time_s_sensor",
                             "x_lsb", "y_lsb", "z_lsb", "ax_g", "ay_g", "az_g",
                             "ax_m_s2", "ay_m_s2", "az_m_s2"))
            while count < samples:
                frames = min(burst_frames, samples - count)
                deadline = time.monotonic() + max(2.0, 4 * frames / odr_hz)
                while sensor.fifo_level() < frames * FIFO_PAYLOAD_BYTES:
                    if time.monotonic() >= deadline:
                        raise TimeoutError("FIFO did not fill: check ACC_CONF_0 and FIFO_CONF")
                    time.sleep(min(0.002, max(0.0005, frames / odr_hz / 4)))
                buffer = sensor.read_fifo_frames(frames)
                for index in range(frames):
                    frame = buffer[index * FIFO_FRAME_BYTES:(index + 1) * FIFO_FRAME_BYTES]
                    tick, raw, xyz = decode_fifo_frame(frame, range_g)
                    if first_tick is None:
                        first_tick = tick
                    else:
                        delta = (tick - previous_tick) % SENSOR_TIME_MODULUS
                        unwrapped_ticks += delta
                        if delta * SENSOR_TICK_SECONDS > 1.5 / odr_hz + SENSOR_TICK_SECONDS:
                            gaps += 1
                    previous_tick = tick
                    writer.writerow((count, tick, unwrapped_ticks * SENSOR_TICK_SECONDS,
                                     *raw, *xyz, *(a * GRAVITY for a in xyz)))
                    count += 1
    finally:
        sensor.close()
    print(f"Saved {count} FIFO frames to {output}; large sensor-time intervals: {gaps}")
    print("Sensor time is local to BMA580; it is not synchronized with your analog DAQ.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=10000)
    parser.add_argument("--odr", type=int, default=800, choices=sorted(ODR_CODES))
    parser.add_argument("--range-g", type=int, default=4, choices=(2, 4, 8, 16))
    parser.add_argument("--output", default="bma580_spi4_data.csv")
    parser.add_argument("--burst-frames", type=int, default=40)
    args = parser.parse_args()
    capture(args.samples, args.odr, args.range_g, args.output, args.burst_frames)
