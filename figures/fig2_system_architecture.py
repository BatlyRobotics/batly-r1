#!/usr/bin/env python3
"""
fig2_system_architecture.py — Figure 2 of the Batly R1 manuscript

Two panels: (a) communication architecture, (b) power architecture. Drawn from a
block layout in matplotlib rather than exported from a drawing tool, so the figure
is reproducible from source and stays consistent with the other figures.

Panel (a) note: the RS485 adapter reaches the host through a USB 3.0 hub, not a
direct port. This is part of the transport path characterised in §4, and is drawn
explicitly rather than simplified away.

Usage:  python3 fig2_system_architecture.py [-o outdir]
"""

import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
from matplotlib.lines import Line2D

BLUE   = "#2E5FD9"
GREEN  = "#21A038"
RED    = "#E0202E"
ORANGE = "#F08000"
INK    = "#1A1D21"
GREY   = "#6B7280"
F_HOST = "#EEF2FB"
F_DRV  = "#E8F4EC"
F_SENS = "#F1F3F5"
F_PWR  = "#FDF0E3"
F_MECH = "#F6EEF6"

plt.rcParams.update({"font.size": 10.5, "savefig.bbox": "tight"})


def box(ax, x, y, w, h, text, fc, ec=INK, lw=1.2, fs=10.0, weight="normal"):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                                boxstyle="round,pad=0.010,rounding_size=0.018",
                                fc=fc, ec=ec, lw=lw, zorder=2))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fs, weight=weight, zorder=3, linespacing=1.4)


def arr(ax, p, q, color=INK, lw=1.3, style="-|>", ls="-", ms=9):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle=style, mutation_scale=ms,
                                 color=color, lw=lw, linestyle=ls, zorder=4,
                                 shrinkA=1.0, shrinkB=1.0))


def lab(ax, x, y, t, fs=9.0, color=GREY, ha="center", va="center", rot=0):
    ax.text(x, y, t, fontsize=fs, color=color, ha=ha, va=va, style="italic",
            rotation=rot, zorder=5, linespacing=1.35)


# ── panel (a): communication ─────────────────────────────────────────────────
def panel_comms(ax):
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
    ax.set_title("(a) Communication architecture", fontsize=12.5,
                 weight="bold", loc="left", pad=6)

    box(ax, 0.015, 0.075, 0.150, 0.855,
        "Raspberry\nPi 5\n\n8 GB RAM\nUbuntu\n24.04 LTS\nROS 2\nJazzy",
        F_HOST, ec=BLUE, lw=1.8, fs=10.4, weight="bold")

    # drive chain
    box(ax, 0.225, 0.788, 0.148, 0.136, "USB 3.0 hub\nUgreen 20290\n4 ports", F_SENS, fs=9.6)
    box(ax, 0.406, 0.788, 0.166, 0.136, "Waveshare\nUSB → RS485 (B)\nWCH CH343G", F_DRV, ec=GREEN, fs=9.6)
    box(ax, 0.605, 0.788, 0.163, 0.136, "ZLTECH\nZLAC8015D V4.0\ndual channel", F_DRV, ec=GREEN, fs=9.6)
    box(ax, 0.801, 0.788, 0.176, 0.136, "2 × ZLLG40ASM100-S\nhub motors\n+ encoders", F_MECH, fs=9.6)
    arr(ax, (0.165, 0.856), (0.225, 0.856), color=BLUE)
    arr(ax, (0.373, 0.856), (0.406, 0.856), color=BLUE)
    arr(ax, (0.572, 0.856), (0.605, 0.856), color=GREEN, style="<|-|>")
    arr(ax, (0.768, 0.856), (0.801, 0.856), color=GREEN, style="<|-|>")
    for xx, tt in ((0.195, "USB 3.0"), (0.390, "USB\ncdc_acm"),
                   (0.588, "RS485\nModbus RTU\n115200 8N1"),
                   (0.784, "3-phase\n+ encoder")):
        ax.text(xx, 0.936, tt, fontsize=8.6, color=GREY, ha="center", va="bottom",
                style="italic", zorder=6, linespacing=1.3,
                bbox=dict(fc="white", ec="none", pad=0.6))

    # lidar, camera
    box(ax, 0.240, 0.640, 0.290, 0.100, "Slamtec RPLIDAR C1\n12 m · 0.72° · 10 Hz", F_SENS, fs=9.6)
    arr(ax, (0.165, 0.690), (0.240, 0.690), color=BLUE)
    lab(ax, 0.202, 0.712, "USB", fs=8.6)

    box(ax, 0.240, 0.500, 0.290, 0.100,
        "ELP USB camera\nglobal shutter · 850 nm IR\n/dev/elp_camera", F_SENS, fs=9.6)
    arr(ax, (0.165, 0.550), (0.240, 0.550), color=BLUE)
    lab(ax, 0.202, 0.572, "USB", fs=8.6)

    # I/O subsystem
    box(ax, 0.240, 0.185, 0.250, 0.245,
        "Innovity Tech\nI/O board\n\nArduino Nano\nRP2040 Connect\n(micro-ROS)\n\n"
        "potentiometer\nbuzzer · 2 switches",
        F_SENS, ec=ORANGE, lw=1.5, fs=9.4)
    arr(ax, (0.165, 0.308), (0.240, 0.308), color=BLUE, style="<|-|>")
    lab(ax, 0.202, 0.372, "USB serial\n/dev/uros", fs=8.6)

    box(ax, 0.520, 0.185, 0.240, 0.245,
        "LSM6DSOX IMU\n(on RP2040 board)\n\n6 × WS2812B\n\nHC-SR04\nultrasonic\n\nservo, 0–180°",
        F_SENS, ec=ORANGE, fs=9.4)
    arr(ax, (0.490, 0.308), (0.520, 0.308), color=ORANGE, style="<|-|>")

    # display
    box(ax, 0.240, 0.050, 0.290, 0.095, "10 in LCD\n(display only, not touch)", F_SENS, fs=9.6)
    arr(ax, (0.165, 0.098), (0.240, 0.098), color=BLUE)
    lab(ax, 0.202, 0.162, "HDMI", fs=8.6)

    ax.add_patch(FancyBboxPatch((0.560, 0.450), 0.425, 0.215,
                                boxstyle="round,pad=0.012,rounding_size=0.015",
                                fc="white", ec=GREY, lw=1.0, ls=(0, (3, 2)), zorder=1))
    lab(ax, 0.772, 0.558,
        "The RS485 adapter reaches the host through a USB 3.0\n"
        "hub rather than a direct port. This hop forms part of the\n"
        "transport path characterised in §4 and is bounded by the\n"
        "0.35 ms per-cycle residual reported in §4.4.1.",
        fs=9.4, color=INK)

    ax.legend(handles=[
        Line2D([], [], color=BLUE, lw=1.6, label="USB / HDMI (host links)"),
        Line2D([], [], color=GREEN, lw=1.6, label="RS485 field bus / motor drive"),
        Line2D([], [], color=ORANGE, lw=1.6, label="microcontroller peripherals")],
        loc="lower right", fontsize=9.4, frameon=True, framealpha=1.0,
        edgecolor="#9AA0A6", fancybox=False, bbox_to_anchor=(1.0, 0.02))


# ── panel (b): power ─────────────────────────────────────────────────────────
def panel_power(ax):
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
    ax.set_title("(b) Power architecture", fontsize=12.5, weight="bold",
                 loc="left", pad=6)

    box(ax, 0.015, 0.500, 0.140, 0.150, "LiFePO₄\n24 V 13 Ah\n(312 Wh)", F_PWR, ec=RED, lw=1.6, fs=9.9)
    box(ax, 0.185, 0.525, 0.090, 0.100, "30 A\nfuse", F_PWR, fs=9.8)
    box(ax, 0.305, 0.500, 0.130, 0.150, "Emergency\nstop\n(interrupts\nall power)", F_PWR, ec=RED, lw=1.6, fs=9.8)
    arr(ax, (0.155, 0.575), (0.185, 0.575), color=RED, lw=1.8)
    arr(ax, (0.275, 0.575), (0.305, 0.575), color=RED, lw=1.8)

    # bus node and the three branches
    ax.plot([0.470, 0.470], [0.185, 0.815], color=RED, lw=1.8, zorder=4)
    arr(ax, (0.435, 0.575), (0.470, 0.575), color=RED, lw=1.8, style="-")
    for y in (0.793, 0.576, 0.218):
        arr(ax, (0.470, y), (0.520, y), color=RED, lw=1.8)

    # branch 1 — 5 V to the host. LA38 #1 enables the board, no power through it.
    box(ax, 0.520, 0.735, 0.220, 0.115,
        "Batly power board\nLM5146, 4-layer\n24 V → 5 V", F_PWR, ec=RED, fs=9.6)
    box(ax, 0.790, 0.735, 0.195, 0.115, "Raspberry Pi 5\nvia USB-C", F_HOST, ec=BLUE, fs=9.8)
    arr(ax, (0.740, 0.793), (0.790, 0.793), color=RED, lw=1.8)
    lab(ax, 0.765, 0.868, "5 V", fs=9.0)
    box(ax, 0.520, 0.895, 0.130, 0.080, "LA38 #1", F_PWR, ec=ORANGE, lw=1.6, fs=9.8)
    arr(ax, (0.585, 0.895), (0.585, 0.850), color=ORANGE, ls=(0, (4, 2)), lw=1.6)
    lab(ax, 0.668, 0.935, "enable", fs=9.0, color=ORANGE, ha="left")

    # branch 2 — 12 V to the display. LA38 #2 carries the power itself.
    box(ax, 0.520, 0.518, 0.130, 0.115, "LA38 #2", F_PWR, ec=ORANGE, lw=1.6, fs=9.8)
    box(ax, 0.695, 0.518, 0.150, 0.115, "24 → 12 V\n5 A converter", F_PWR, ec=RED, fs=9.6)
    box(ax, 0.888, 0.518, 0.097, 0.115, "10 in\nLCD", F_SENS, fs=9.8)
    arr(ax, (0.650, 0.576), (0.695, 0.576), color=RED, lw=1.8)
    arr(ax, (0.845, 0.576), (0.888, 0.576), color=RED, lw=1.8)
    lab(ax, 0.866, 0.652, "12 V", fs=9.0)

    # branch 3 — 24 V to the motor controller. LA38 #3 drives the relay coil.
    box(ax, 0.520, 0.160, 0.220, 0.115, "24 V 40 A relay\n(4-pin)", F_PWR, ec=RED, fs=9.6)
    box(ax, 0.790, 0.160, 0.195, 0.115, "ZLAC8015D\n→ hub motors", F_DRV, ec=GREEN, fs=9.8)
    arr(ax, (0.740, 0.218), (0.790, 0.218), color=RED, lw=1.8)
    lab(ax, 0.765, 0.293, "24 V", fs=9.0)
    box(ax, 0.520, 0.028, 0.130, 0.080, "LA38 #3", F_PWR, ec=ORANGE, lw=1.6, fs=9.8)
    arr(ax, (0.585, 0.108), (0.585, 0.158), color=ORANGE, ls=(0, (4, 2)), lw=1.6)
    lab(ax, 0.668, 0.068, "relay coil", fs=9.0, color=ORANGE, ha="left")

    ax.add_patch(FancyBboxPatch((0.020, 0.120), 0.420, 0.200,
                                boxstyle="round,pad=0.012,rounding_size=0.015",
                                fc="white", ec=GREY, lw=1.0, ls=(0, (3, 2)), zorder=1))
    lab(ax, 0.230, 0.222,
        "One LA38 switch per branch. Only #2 carries power,\n"
        "feeding the 12 V converter directly. #1 and #3 act as\n"
        "control signals instead — an enable to the 5 V board\n"
        "and the coil of the 40 A relay — so neither passes\n"
        "load current.",
        fs=9.4, color=INK)

    ax.legend(handles=[
        Line2D([], [], color=RED, lw=1.8, label="power"),
        Line2D([], [], color=ORANGE, lw=1.4, ls=(0, (4, 2)), label="control signal")],
        loc="upper left", fontsize=9.4, frameon=True, framealpha=1.0,
        edgecolor="#9AA0A6", fancybox=False, bbox_to_anchor=(0.02, 0.98))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--outdir", default="wordfigs")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    fig, ax = plt.subplots(2, 1, figsize=(12.2, 13.8))
    panel_comms(ax[0]); panel_power(ax[1])
    fig.tight_layout(h_pad=3.0)
    fig.savefig(os.path.join(a.outdir, "fig2_system_architecture.png"), dpi=300)
    fig.savefig(os.path.join(a.outdir, "fig2_system_architecture.pdf"))
    print("written to", a.outdir)
