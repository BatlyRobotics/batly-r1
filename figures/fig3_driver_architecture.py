#!/usr/bin/env python3
"""
fig3_driver_architecture.py — Figure 3 of the Batly R1 manuscript

Driver architecture: the two independent ROS 2 wall timers, the Modbus transaction
sequence within each, and the interfaces. Drawn to scale-free block layout rather
than exported from a drawing tool so that the figure is reproducible from source.

The structural point of the figure is that command transmission and feedback
acquisition are separate timers at separate rates, serialised by a single-threaded
executor — not one callback doing both.

Usage:  python3 fig3_driver_architecture.py [-o outdir]
"""

import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

BLUE   = "#2E5FD9"
GREEN  = "#21A038"
RED    = "#E0202E"
ORANGE = "#F08000"
INK    = "#1A1D21"
GREY   = "#6B7280"
FILL_N = "#EEF2FB"      # node interior
FILL_C = "#E8F4EC"      # command path
FILL_O = "#FDF0E3"      # odometry path
FILL_H = "#F1F3F5"      # host / hardware

plt.rcParams.update({
    "font.size": 9.5,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
    "savefig.bbox": "tight",
})


def box(ax, x, y, w, h, text, fc, ec=INK, lw=1.2, fs=9.5, weight="normal",
        style="round,pad=0.012,rounding_size=0.02", ha="center"):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=style,
                                fc=fc, ec=ec, lw=lw, zorder=2))
    ax.text(x + w / 2 if ha == "center" else x + 0.012, y + h / 2, text,
            ha=ha, va="center", fontsize=fs, zorder=3, weight=weight,
            linespacing=1.45)


def arrow(ax, p, q, color=INK, lw=1.4, style="-|>", ls="-", rad=0.0, ms=9):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle=style, mutation_scale=ms,
                                 color=color, lw=lw, linestyle=ls, zorder=4,
                                 connectionstyle=f"arc3,rad={rad}",
                                 shrinkA=1.5, shrinkB=1.5))


def label(ax, x, y, t, fs=8.5, color=GREY, ha="center", va="center",
          style="italic", weight="normal"):
    ax.text(x, y, t, fontsize=fs, color=color, ha=ha, va=va, style=style,
            weight=weight, zorder=5, linespacing=1.4)


def build():
    fig, ax = plt.subplots(figsize=(9.8, 9.4))
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")

    # ── ROS 2 graph, above the node ──────────────────────────────────────────
    box(ax, 0.055, 0.930, 0.245, 0.058,
        "/cmd_vel\ngeometry_msgs/Twist", FILL_H, ec=BLUE, fs=8.8)
    box(ax, 0.375, 0.930, 0.250, 0.058,
        "10 services\n(enable, e-stop, faults, status, …)", FILL_H, ec=BLUE, fs=8.3)
    box(ax, 0.700, 0.930, 0.245, 0.058,
        "/raw_odom\nnav_msgs/Odometry", FILL_H, ec=BLUE, fs=8.8)

    # ── the node ─────────────────────────────────────────────────────────────
    ax.add_patch(FancyBboxPatch((0.035, 0.290), 0.93, 0.605,
                                boxstyle="round,pad=0.008,rounding_size=0.02",
                                fc=FILL_N, ec=INK, lw=1.8, zorder=1))
    ax.text(0.108, 0.866, "zlac_node", fontsize=11.5, weight="bold", zorder=3)
    ax.text(0.108, 0.840,
            "rclpy · single-threaded executor · pymodbus",
            fontsize=8.8, color=GREY, style="italic", zorder=3)

    # subscription callback
    box(ax, 0.055, 0.740, 0.245, 0.080,
        "cmd_vel_callback\nstore latest Twist\nand arrival time",
        "white", ec=BLUE, fs=8.6)

    # ── command path ─────────────────────────────────────────────────────────
    box(ax, 0.055, 0.620, 0.245, 0.065,
        "COMMAND TIMER\ncmd_rate_hz = 20 Hz", FILL_C, ec=GREEN, lw=1.6,
        fs=9.0, weight="bold")
    box(ax, 0.055, 0.435, 0.245, 0.135,
        "differential kinematics\n"
        "$v_{L,R} = v \\mp \\omega L/2$\n"
        "→ integer RPM (1 RPM floor)\n"
        "command timeout → zero", FILL_C, ec=GREEN, fs=8.6)
    box(ax, 0.055, 0.310, 0.245, 0.075,
        "FC 0x10 write\n0x2088 / 0x2089\nboth channels, one frame",
        "white", ec=GREEN, fs=8.6)

    # ── odometry path ────────────────────────────────────────────────────────
    box(ax, 0.700, 0.620, 0.245, 0.065,
        "ODOMETRY TIMER\nodom_rate_hz = 50 Hz", FILL_O, ec=ORANGE, lw=1.6,
        fs=9.0, weight="bold")
    box(ax, 0.700, 0.310, 0.245, 0.075,
        "FC 0x03 read × 2\n0x20A7 (L), 0x20A9 (R)\n32-bit encoder position",
        "white", ec=ORANGE, fs=8.6)
    box(ax, 0.700, 0.435, 0.245, 0.135,
        "position differencing\n→ wheel velocity\n"
        "pose integration → Odometry\n+ explicit covariance\n"
        "(0x20AB velocity register\nnot read)",
        FILL_O, ec=ORANGE, fs=8.6)

    # ── parameters ───────────────────────────────────────────────────────────
    box(ax, 0.345, 0.310, 0.310, 0.260,
        "PARAMETERS (ROS 2 YAML)\n\n"
        "cmd_rate_hz · odom_rate_hz\n"
        "cmd_timeout · mb_timeout_ms\n"
        "wheel_radius · wheel_base\n"
        "ticks_per_rev · gear_ratio\n"
        "max_rpm · flip flags\n"
        "frame ids · device · baud · slave",
        "white", ec=GREY, lw=1.1, fs=8.4)

    # ── mutual exclusion note between the two timers ─────────────────────────
    arrow(ax, (0.302, 0.700), (0.698, 0.700), color=RED, lw=1.3,
          style="<|-|>", ls=(0, (4, 2)), ms=8)
    label(ax, 0.500, 0.762,
          "single-threaded executor:\n"
          "the two timer callbacks are mutually\n"
          "exclusive and cannot interleave",
          fs=8.4, color=RED)

    # ── vertical flow arrows ─────────────────────────────────────────────────
    arrow(ax, (0.078, 0.930), (0.078, 0.821), color=BLUE)          # /cmd_vel in
    arrow(ax, (0.178, 0.740), (0.178, 0.686), color=GREEN)         # cb -> cmd timer
    arrow(ax, (0.178, 0.620), (0.178, 0.571), color=GREEN)
    arrow(ax, (0.178, 0.435), (0.178, 0.386), color=GREEN)
    arrow(ax, (0.822, 0.620), (0.822, 0.571), color=ORANGE)        # timer -> read
    arrow(ax, (0.900, 0.385), (0.900, 0.434), color=ORANGE)        # read -> integrate
    ax.plot([0.700, 0.682], [0.502, 0.502], color=ORANGE, lw=1.4, zorder=4)
    arrow(ax, (0.682, 0.502), (0.682, 0.930), color=ORANGE)        # /raw_odom out
    arrow(ax, (0.500, 0.895), (0.500, 0.930), color=BLUE, style="<|-|>")                                            # services


    # ── transport layer ──────────────────────────────────────────────────────
    box(ax, 0.055, 0.185, 0.890, 0.065,
        "pymodbus RTU client  →  /dev/rs485  (udev symlink to /dev/ttyACM1)",
        FILL_H, ec=GREY, lw=1.2, fs=9.0)
    arrow(ax, (0.178, 0.310), (0.178, 0.251), color=GREEN)
    arrow(ax, (0.822, 0.251), (0.822, 0.310), color=ORANGE)

    box(ax, 0.055, 0.070, 0.890, 0.080,
        "Waveshare USB TO RS485 (B) · WCH CH343G · cdc_acm\n"
        "RS485 · Modbus RTU · 115200 baud · 8N1  →  ZLTECH ZLAC8015D V4.0",
        FILL_H, ec=INK, lw=1.4, fs=9.0)
    arrow(ax, (0.500, 0.185), (0.500, 0.151), color=INK, style="<|-|>", ms=8)

    label(ax, 0.500, 0.028,
          "two Modbus transactions per odometry cycle; the mandated inter-frame "
          "silence dominates the cycle time (§4.4.1)",
          fs=8.6, color=INK)

    ax.set_title("", pad=2)
    return fig


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--outdir", default="wordfigs")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    f = build()
    f.savefig(os.path.join(a.outdir, "fig3_driver_architecture.png"), dpi=300)
    f.savefig(os.path.join(a.outdir, "fig3_driver_architecture.pdf"))
    print("written to", a.outdir)
