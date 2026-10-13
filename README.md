# Panda Eye-in-Hand Grasping (AI for Autonomous Systems – A4)

A Franka Panda arm in PyBullet finds fruits on a table with only its hand-mounted RGB-D camera
and puts them on a plate.

**Pipeline:** exploration poses → RGB-D to point cloud → ICP merge → RANSAC plane removal →
DBSCAN clustering → YOLOv8 labelling → crane-grasp pick and place.

- Code: `A4/src/control.py`, `A4/src/perception.py`, `A4/src/planning.py`
- Full explanation, how to run, and viva Q&A: [VIVA_NOTES.md](VIVA_NOTES.md)

Quick run (Linux, course environment):
```
cd A4 && conda env create -f a4_2026.yml && conda activate a4_2026
cd src && python main.py
```
