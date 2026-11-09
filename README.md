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
