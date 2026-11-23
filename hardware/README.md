# Hardware

KiCad 10 projects for the synth family's circuit boards, and the shared parts
library they all use. The firmware and the browser prototype are in the folders
above this one.

## What is here

| Path | What it is |
|---|---|
| `hub-main/` | MIDI clock hub main board: Teensy 4.1, multiplexer, MIDI in and four outs, power. Mostly through-hole, hand-soldered, chips in sockets. |
| `led-tile/` | One-LED tile, 12 × 10 × 0.8 mm: a reverse-mount SK6812 MINI-E, its 100 nF capacitor, a series resistor on data in, and IN and OUT wire pads (+5 V, data, GND). One design for the hub, the synth and later boxes, ordered as maker-assembled snap-apart sheets. Design notes in `led-tile/README.md`. |
| `lib/Synth.kicad_sym` | Shared symbol library, `Synth` in every project. |
| `lib/Synth.pretty/` | Shared footprint library, `Synth` in every project. |
| `lib/3dmodels/` | STEP models for the shared footprints. |

Panel drawings and the steel DXF come from the millimetre layout files in
`../layouts/`, not from KiCad.

## One folder per board

Each board is its own KiCad project in its own folder, named the same as the
folder: `hub-main/hub-main.kicad_pro`. A new board gets a new folder next to
these, such as `synth-main/`. Keep the same pattern so the ignore rules and
library paths keep working.

Each project's `sym-lib-table` and `fp-lib-table` add the shared library as
`${KIPRJMOD}/../lib/...`. `${KIPRJMOD}` means "this project's folder", so the
paths work wherever the repo is cloned, on the Windows PC or the Mac. Never
point a project at a library by its full `E:\...` path.

## Where parts come from

1. **KiCad's own libraries** (installed with KiCad) for common parts:
   resistors, capacitors, diodes, sockets, headers, 74-series and op-amp chips.
2. **The `Synth` library** for anything KiCad doesn't have or that we've
   changed: the Teensy 4.1, panel pots and selectors, TRS jacks, the DC jack,
   the LED tile's pads. Make the part once here and every board can use it.

Don't edit a part inside one board's schematic or board file to "fix" it.
Fix it in the library, then update it in each board
(Tools > Update Symbols from Library, Tools > Update Footprints from Library).

## What is kept in git

Kept: `*.kicad_pro`, `*.kicad_sch`, `*.kicad_pcb`, `sym-lib-table`,
`fp-lib-table` and the library files.

Ignored (see `../.gitignore`):
- `*.kicad_prl`: your window layout, which layers are visible, and so on.
- `.history/`: KiCad's automatic local snapshots (File > Local History).
- `*-backups/`: zipped automatic backups.
- Lock files and caches.
- `out/`: generated outputs.

Fabrication outputs (Gerbers, drill files, BOM, placement files, PDFs) go to
the board's own `out/` folder, which git ignores. When a board is ordered, tag
the commit (for example `hub-main-rev-A`) and attach the exact files that were
sent to the maker to a GitHub release for that tag. That way, what was built is
always findable, and the repo doesn't fill up with old Gerbers.

## Working routine

1. Before starting: `git pull`, so you have the latest from the Mac or GitHub.
2. Work in KiCad. Save often. Local History has snapshots if you need to step back.
3. Close the schematic and board editors before committing, so nothing is half-saved.
4. Commit with a message saying what changed ("hub-main: MIDI out buffer and jacks").
5. `git push`. On the first push, a browser window asks you to sign in to GitHub.

Changes to a board go on a branch and reach `main` through a pull request, the
same as the firmware. A push to `main` re-runs the firmware CI and republishes
the prototype page.

## Board revisions

Each board's title block holds its revision letter (File > Page Settings in
both editors). Rev A is the first one ordered. Once a revision is ordered,
bump the letter before changing anything, so the files never disagree with a
board you're holding.
