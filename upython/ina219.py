# Minimal INA219 driver for MicroPython
# Focus: bus_voltage, system_draw_amp, system_power_watt, batt_time_remaining
# Shunt: 20A/75mV -> Rshunt = 0.00375 Ω

from machine import I2C
import time

class INA219:
    _REG_CONFIG      = 0x00
    _REG_SHUNT_V     = 0x01
    _REG_BUS_V       = 0x02
    _REG_CALIB       = 0x05

    def __init__(self, i2c: I2C, addr=0x40, r_shunt=0.00375):
        self.i2c = i2c
        self.addr = addr
        self.r_shunt = r_shunt

        # Config: 32V range, PGA /8 (±320mV), 12-bit bus+shunt, continuous
        # Bits: BRNG=1, PGA=3, BADC=0xF, SADC=0xF, MODE=0x7
        config = (1 << 13) | (3 << 11) | (15 << 7) | (15 << 3) | 7  # 0x3FFF
        self._w16(self._REG_CONFIG, config)

        # Optional: write a reasonable calibration (not required since we compute I from Vshunt/R)
        # Using Imax=20A -> Current_LSB ≈ 0.00061037A -> Cal ≈ 17895
        self._w16(self._REG_CALIB, 17895)

    # ---------- low-level ----------
    def _w16(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([(val >> 8) & 0xFF, val & 0xFF]))

    def _r16(self, reg):
        b = self.i2c.readfrom_mem(self.addr, reg, 2)
        # signed for shunt, unsigned for others; caller handles interpretation
        return (b[0] << 8) | b[1]

    # ---------- measurements ----------
    def bus_voltage(self) -> float:
        """Volts (Bus: 4mV/bit, bits [15:3])."""
        raw = self._r16(self._REG_BUS_V)
        return ((raw >> 3) & 0x1FFF) * 0.004

    def _shunt_voltage(self) -> float:
        """Volts (Shunt: 10µV/bit, signed)."""
        raw = self._r16(self._REG_SHUNT_V)
        if raw & 0x8000:  # sign
            raw -= 1 << 16
        return raw * 10e-6  # 10 µV/bit

    def system_draw_amp(self) -> float:
        """Amps (positive for discharge/load draw)."""
        v_shunt = self._shunt_voltage()
        return v_shunt / self.r_shunt

    def system_power_watt(self) -> float:
        """Watts (based on bus voltage * |current|)."""
        v = self.bus_voltage()
        i = abs(self.system_draw_amp())
        return v * i

    def batt_time_remaining(self, capacity_Wh: float, soc: float = 1.0) -> str:
        """
        Returns HH:mm estimate assuming steady load.
        capacity_Wh: battery nominal capacity in watt-hours (e.g., 36Wh)
        soc: state of charge fraction (0..1). If unknown, leave 1.0 for "full".
        """
        p = self.system_power_watt()
        if p <= 1e-3:  # ~0W draw -> effectively infinite
            return "∞"
        remaining_Wh = max(0.0, capacity_Wh * min(max(soc, 0.0), 1.0))
        hours = remaining_Wh / p
        if hours > 999:  # cap display
            return "∞"
        total_min = int(hours * 60)
        hh, mm = divmod(total_min, 60)
        return f"{hh:02d}:{mm:02d}"

# ---------- example ----------
# from machine import Pin, I2C
# i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)  # adjust pins/port for your board
# ina = INA219(i2c, addr=0x40, r_shunt=0.00375)
# CAP_WH = 36.0   # e.g., 12V 3Ah pack
# SOC = 0.75      # 75% remaining (if known)
# while True:
#     v = ina.bus_voltage()
#     i = ina.system_draw_amp()
#     w = ina.system_power_watt()
#     eta = ina.batt_time_remaining(CAP_WH, SOC)
#     print(f"V={v:.2f}V  I={i:.3f}A  P={w:.2f}W  ETA={eta}")
#     time.sleep(1)
