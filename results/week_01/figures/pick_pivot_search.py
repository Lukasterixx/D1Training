"""Figure for the pivoting cup search (Week 1 log, 2026-09-17): what the survey pose sees, before and after.

    python results/week_01/figures/pick_pivot_search.py OLD_SURVEY_FRAME NEW_SURVEY_FRAME PIVOT_FRAME PICK_MP4

Frames are the annotated wrist images a pick run writes to frames/; the MP4 is a run's pick.mp4, whose right
640 px are the overview camera. The logs are not committed, so the paths are recorded in the week log.
"""
import sys
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def rgb(path):
    return cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)


def last_overview(mp4):
    video = cv2.VideoCapture(str(mp4))
    frame = None
    while True:
        ok, image = video.read()
        if not ok:
            break
        frame = image
    video.release()
    return cv2.cvtColor(frame[:, -640:], cv2.COLOR_BGR2RGB)


def main():
    old, new, pivot, mp4 = sys.argv[1:5]
    panels = [
        (rgb(old), "Survey over the back (before): the head hides a cup 0.42 m ahead"),
        (rgb(new), "Survey past the head (after): the same cup, seen and fitted"),
        (rgb(pivot), "Pivot stop: a cup 40° left, outside the straight-ahead frame"),
        (last_overview(mp4), "Overview: the cup found at the pivot stop, lifted"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(11, 8.6))
    for ax, (image, title) in zip(axes.flat, panels):
        ax.imshow(image)
        ax.set_title(title, fontsize=10)
        ax.axis("off")
    fig.tight_layout()
    out = Path(__file__).with_suffix(".png")
    fig.savefig(out, dpi=110)
    print(out)


if __name__ == "__main__":
    main()
