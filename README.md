# agx modular snake

Repository for modular snake project in Mechatronics and System Integration course MMA4004

run this command

``
$projectRoot = (Resolve-Path ..).Path
$env:PYTHONPATH = "$projectRoot;$env:PYTHONPATH"
& "$env:PYTHONHOME\python.exe" .\slope_scene.py
``

Fast run without the viewer, for tuning the gaits (from the project root; prints a summary per snake):

``
& "$env:PYTHONHOME\python.exe" -B .\environment\headless_course.py --configs PPYPP PYPYP --scene hills --time 600
``

Scenes: `hills`, `maze`, `course`, `flat`. Add `--wheels` to give YYPYY passive wheels (serpentine needs them).
Other options: `--design v3` (the earlier robot), `--head-mass` / `--tail-mass` (kg), `--friction` (track), `--cradles`.

## Robot design

The simulation builds the V5 design (`environment/assets/ssv5`) by default; set `DESIGN = 'v3'` in
`snakeapp/snake_module.py` for the earlier bare modules. Assembly, from the front:

- head (`SS_V5_head`) bolted to the first module's head arm (`SS_V5_arm_head`), so the snake leads with an arm
- five modules: cradle (`SS_V5_krybbe_kort`, or `_lang` per module with `cradles`), Parallax servo, arm (`SS_V5_arm_vanlig`)
- a plate (`SS_V5_mellomplate`) between each pair of modules, with a tooth under each bottom corner
- tail (`SS_V5_tail`) bolted to the last module's cradle
- teeth (`SS_V5_tenner`) under the four bottom corners of the head and of the tail; the notched
  `_head_L` / `_head_R` ones at the head's front corners

The snake stands on the tips of the teeth, 5 mm below the modules. After changing the CAD files, run
`python snakeapp/convert_snake_to_mesh.py v5` to remake the meshes the simulation loads. The head and
tail masses (`HEAD_MASS`, `TAIL_MASS` in `snake_module.py`) default to printed plastic only; set them
to the weighed values, electronics and battery included.
