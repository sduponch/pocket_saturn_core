# Sega Saturn for the Analogue Pocket

An openFPGA core that plays Sega Saturn games on the Analogue Pocket. It is a port of the MiSTer Saturn core,
[Saturn_MiSTer](https://github.com/MiSTer-devel/Saturn_MiSTer) by Sergiy Dvodnenko (srg320) and contributors,
trimmed to fit the Pocket's Cyclone V FPGA and adapted to the Pocket's memories and the Analogue Pocket Framework.

## Analogizer: JVS, light guns, wheel and analog video (branch `jvs`)

This branch adds the [Analogizer](https://github.com/RndMnkIII/Analogizer) adapter on the cartridge port: an arcade
cabinet's JVS I/O board through the JVS SNAC adapter, and the adapter's analog video out. Tested on an Analogue Pocket
with a Sega 838-13683B board (pads, Virtua Cop with guns and recoil) and a Namco NA-JV (Time Crisis 4 cabinet); the
3D pad / wheel modes are not tested on a cabinet yet.

### Using it

Core Settings:

| Setting | Choices |
|---|---|
| JVS (Analogizer) | Off, Pads, P1 gun, P1 + P2 guns, P1 3D pad, P1 wheel |
| JVS recoil | Off, Single (one 50 ms pulse a shot), Auto (50 ms on / 150 ms off while the trigger is held) |
| Analogizer video | Off, RGBS (15 kHz), RGBHV (31 kHz, line doubled, no scanlines) |
| JVS Profile (data slot) | a `jvs_*.bin` profile of `Assets/saturn/akoaysigod.Saturn/` |

- **Init**: when JVS is on, the core resets the board twice a second apart, waits 3 s (a Sega 838-13683B ignores an
  address sent sooner), sets address 1, reads the identity and features, sends the profile's MAINID and vendor frames
  (a Namco NA-JV drives its outputs only after them), then polls every 14 ms. If the board stops answering for
  0.5 s, port 1 goes back to the Pocket's controls and the init starts again.
- **Pads**: player 1 on port 1, player 2 on port 2. Push 1 2 3 = A B C, push 4 5 6 = X Y Z, push 7 8 = L R,
  Start = Start (Service is not used). Opposite directions cancel.
- **Guns** (Virtua Gun / Stunner): the profile's axes give the aim (absolute position), its trigger mask the trigger,
  Start the gun's Start. Aiming off screen reloads, as on a real gun. Calibrate in the game's options.
- **3D pad / wheel** (port 1): the profile's P1 X (steering), Y (accelerator) and Z (brake). The axes calibrate
  themselves: keep the wheel centred and the pedals released while the board starts, then turn the wheel fully and
  press each pedal once. 3D pad: X on the stick, accelerator and brake on the analog R / L triggers. Arcade Racer
  (wheel): the pedals become B (accelerator) and A (brake) past half way.
- **Recoil**: made from the trigger (console games never drive one): OUTPUT1 = the profile's base value, plus the
  player's recoil mask during the pulse.
- **Profiles** (same 256-byte format and files as the other Analogizer JVS cores, see the jvs library's
  `scripts/jvs_vendor.py`): `jvs_sega_recoil_test.bin` (Sega board: push 1 = trigger, recoil on outputs 1 / 2),
  `jvs_namco_timecrisis4.bin` (Namco NA-JV), `jvs_initiald8.bin` (Initial D8's 837-14572: steering CH0, accelerator
  CH1, brake CH2), `jvs_gun_p1.bin` (axes CH0 / CH1, push 1, no recoil). Loading a profile from the menu does not
  reset the game.
- **Analogizer video**: the pins of the other Analogizer cores (bank 3 R + syncs, bank 2 G + blank, bank 1 B + DAC
  clock). RGBS: composite sync on HSYNC. RGBHV: separate negative syncs, 31 kHz; interlaced modes come out as
  line-doubled fields.

### What changed in this port

Added:
- `src/fpga/glue/an_jvs.sv`: the JVS host of MIRLO64 (framing, escapes, checksums, the poll resent by itself), with a
  bit divisor that follows `clk_sys` (it moves from 53.4 to 57.3 MHz with the video mode).
- `src/fpga/glue/sat_jvs.sv`: its registers on the CD block CPU's EXT port (words 0x78 / 0x7C), the pads, guns and
  analog values the firmware writes, the menu settings, and the whole JVS profile in a dual-clock RAM.
- `src/fw/cdb/smpc_jvs.c`: the JVS init and poll, pads, guns, 3D pad / wheel calibration and recoil, on the CD block
  CPU beside the SMPC program (called from `poll_run`, on the SMPC's 4 MHz time base: the CD block's timer is held at 0
  while the SMPC keeps the CD block in reset).
- `src/fpga/rtl/lightgun2.sv`: the two Virtua Guns' sensors (see Simplified).
- `src/fpga/glue/sat_avideo.sv`: the Analogizer video out and its line doubler.
- Benches in `src/fpga/sim/jvs/` (Questa): `tb_sat_jvs.sv` (the JVS block against a behavioural I/O board),
  `tb_lg2.sv` (the gun sensors against Saturn_MiSTer's `lightgun.sv`), `tb_sat_avideo.sv` (the line doubler).

Modified:
- `core.json`: `cartridge_adapter` 0 (with -1 the Pocket keeps the cartridge port off and the adapter never answers).
- `rtl/hps2pad.sv`: port 2 can be a pad; Virtua Gun on either port; 3D pad / Arcade Racer handshake on port 1.
- `rtl/Saturn/CD/CDB_HLE.sv`, `rtl/Saturn/Saturn.sv`: an `EXT_RE` read strobe on the EXT port (a register read that
  moves a pointer on); the firmware RAM's second bank grown from 2048 to 4096 words (+8 M10K) for the JVS code.
- `src/fw/cdb/build.sh`: the image size (24576 words) to match.
- `pocket_rtl/pk_settings.sv`, `pocket_rtl/saturn_top.sv`, `core/core_top.v`: the new settings, the cartridge pins,
  the profile's data slot kept out of the load reset; `core/core_constraints.sdc`: the new clock crossings.
- `interact.json`: the three settings above; `data.json`: the JVS Profile slot (id 5, bridge 0x50000000).

Removed:
- Disc slot `File 25` (id 40): APF allows 32 data slots and the profile needed one, so a game has **24 .bin files at
  most** (merge the tracks of a bigger rip into one .bin; `HOW_TO_ADD_GAMES.txt` and `instance-packager.json` updated).
- From MIRLO64's JVS host: the error counter (STAT[23:16] reads 0).

Simplified, to fit (the FPGA is at 98-99 %):
- Saturn_MiSTer's `lightgun.sv` (one per gun) became `lightgun2.sv`: one set of raster counters for both guns, only
  the sensor (no mouse, crosshair, buttons or auto-reload), and the aim's conversion (pixel, line, off screen) done by
  the firmware. Its output matches two `lightgun.sv` cycle for cycle (`tb_lg2.sv`).
- Saturn_MiSTer's HPS2PAD peripherals: only the digital pad, the Virtua Gun and the 3D pad / Arcade Racer (port 1);
  no mouse, keyboard or mission stick.
- Analogizer video: RGBS and RGBHV only (no RGsB, YPbPr, Y/C or scanlines), with a minimal line doubler (a two-line
  buffer read twice at twice the pixel rate) instead of the Analogizer cores' scandoubler.

Building notes:
- The SMPC program `src/fpga/rtl/Saturn/SMPC/smpc.mif` is not in this repository (`.gitignore`): take it from
  Saturn_MiSTer `rtl/Saturn/SMPC/smpc.mif` (commit 9aa9449).
- The firmware image (`build.sh SMPC_FW=1 CD_SRC=1`) was built with Ubuntu's clang 22 and lld 20
  (`LLVM=<clang 22 bin> LD_LLD=<lld 20>`); clang 20 / 21 produce code too big for the RAM.
- Timing: the stock core already misses timing on `clk_ram` with this Quartus (25.1 Lite); the JVS builds do too
  (up to a few hundred ps on `clk_ram`), as the stock one.

## Licenses

This repository is distributed under the GNU General Public License, version 3 or (at your option) any later
version: see `LICENSE`. Files from other projects keep their own notices:

| Path | From | License |
|---|---|---|
| `src/fpga/rtl` | Saturn_MiSTer (commit 9aa9449, 2026-10-19), modified | as upstream (upstream has no license file) |
| `src/fpga/rtl/FX68K` | fx68k by Jorge Cwik, modified | GPL-3.0-or-later |
| `src/fpga/rtl/Saturn/CD/femtorv` | FemtoRV32 by Bruno Levy and Matthias Koch | BSD-3-Clause (`LICENSE` there) |
| `src/fpga/rtl/Saturn/CD/picorv32.v` | PicoRV32 by Claire Xenia Wolf | ISC (in the file) |
| `src/fw/cdb/cdb_hle.c` | Mednafen's CD block emulation (Mednafen Team), adapted | GPL-2.0-or-later |
| `src/fpga/apf` | Analogue Pocket Framework (Analogue); Intel / Altera generated files | as in each file |
| `src/fpga/core/core_top.v`, `core_bridge_cmd.v` | Analogue's openFPGA core template, modified | as the template |
| `src/fpga/core/sound_i2s.sv`, `sync_fifo.sv` | Adam Gastineau | MIT (in the files) |
| `src/fpga/glue/an_jvs.sv`, `src/fpga/sim/jvs/jvs_dev.sv` | MIRLO64 (Sébastien Duponcheel), modified | as MIRLO64 |
| `src/fpga/rtl/lightgun2.sv`, `src/fpga/sim/jvs/lightgun_ref.sv` | Saturn_MiSTer `rtl/lightgun.sv` (commit 964714a), modified | as upstream |
| `dist/Platforms/_images/saturn.bin`, `dist/Cores/akoaysigod.Saturn/icon.bin` | photo of a Saturn by Evan-Amos, [Wikimedia Commons](https://commons.wikimedia.org/wiki/File:Sega-Saturn-Console-Set-Mk1.jpg), converted with `tools/png2pocket.py` | public domain |
| everything else | this project (akoaysigod) | GPL-3.0-or-later |

Sega Saturn is a trademark of Sega. This project is not affiliated with Sega, Analogue or the MiSTer project.
