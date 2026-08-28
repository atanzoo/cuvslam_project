#!/usr/bin/env python3
"""Create a near-field observable variant of the locked mapping world."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


INSERT_BEFORE = '    <model name="slam_bot">\n'

FEATURE_BLOCK = """    <!--
      Diagnostic-only near-field features for stereo forward-motion
      observability. They are visual-only and do not change collision geometry.
    -->
    <model name="near_field_observability_boards">
      <static>true</static>
      <link name="link">
        <visual name="left_board">
          <pose>-0.30 -0.75 0.55 0 0 0</pose>
          <geometry><box><size>0.05 0.95 0.95</size></box></geometry>
          <material><ambient>0.88 0.88 0.88 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
        <visual name="left_black_a">
          <pose>-0.328 -0.98 0.34 0 0 0</pose>
          <geometry><box><size>0.012 0.28 0.24</size></box></geometry>
          <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>
        </visual>
        <visual name="left_black_b">
          <pose>-0.328 -0.58 0.68 0 0 0</pose>
          <geometry><box><size>0.012 0.22 0.32</size></box></geometry>
          <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>
        </visual>
        <visual name="left_black_c">
          <pose>-0.328 -0.90 0.90 0 0 0</pose>
          <geometry><box><size>0.012 0.16 0.12</size></box></geometry>
          <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>
        </visual>

        <visual name="right_board">
          <pose>0.20 -2.65 0.55 0 0 0</pose>
          <geometry><box><size>0.05 0.90 0.95</size></box></geometry>
          <material><ambient>0.18 0.18 0.18 1</ambient><diffuse>0.22 0.22 0.22 1</diffuse></material>
        </visual>
        <visual name="right_white_a">
          <pose>0.172 -2.87 0.30 0 0 0</pose>
          <geometry><box><size>0.012 0.24 0.18</size></box></geometry>
          <material><ambient>0.98 0.98 0.98 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
        <visual name="right_white_b">
          <pose>0.172 -2.48 0.58 0 0 0</pose>
          <geometry><box><size>0.012 0.30 0.20</size></box></geometry>
          <material><ambient>0.98 0.98 0.98 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
        <visual name="right_white_c">
          <pose>0.172 -2.78 0.86 0 0 0</pose>
          <geometry><box><size>0.012 0.18 0.16</size></box></geometry>
          <material><ambient>0.98 0.98 0.98 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>

        <!-- Northbound leg: boards face -Y. -->
        <visual name="north_inner_board">
          <pose>0.75 -0.10 0.55 0 0 0</pose>
          <geometry><box><size>0.95 0.05 0.95</size></box></geometry>
          <material><ambient>0.86 0.86 0.86 1</ambient><diffuse>0.94 0.94 0.94 1</diffuse></material>
        </visual>
        <visual name="north_inner_black_a">
          <pose>0.52 -0.128 0.34 0 0 0</pose>
          <geometry><box><size>0.26 0.012 0.22</size></box></geometry>
          <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>
        </visual>
        <visual name="north_inner_black_b">
          <pose>0.92 -0.128 0.70 0 0 0</pose>
          <geometry><box><size>0.20 0.012 0.30</size></box></geometry>
          <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>
        </visual>
        <visual name="north_outer_board">
          <pose>2.65 0.30 0.55 0 0 0</pose>
          <geometry><box><size>0.90 0.05 0.95</size></box></geometry>
          <material><ambient>0.16 0.16 0.16 1</ambient><diffuse>0.20 0.20 0.20 1</diffuse></material>
        </visual>
        <visual name="north_outer_white_a">
          <pose>2.43 0.272 0.32 0 0 0</pose>
          <geometry><box><size>0.24 0.012 0.18</size></box></geometry>
          <material><ambient>0.98 0.98 0.98 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
        <visual name="north_outer_white_b">
          <pose>2.82 0.272 0.67 0 0 0</pose>
          <geometry><box><size>0.28 0.012 0.24</size></box></geometry>
          <material><ambient>0.98 0.98 0.98 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>

        <!-- Westbound leg: boards face +X. -->
        <visual name="west_inner_board">
          <pose>0.30 0.75 0.55 0 0 0</pose>
          <geometry><box><size>0.05 0.95 0.95</size></box></geometry>
          <material><ambient>0.84 0.84 0.84 1</ambient><diffuse>0.92 0.92 0.92 1</diffuse></material>
        </visual>
        <visual name="west_inner_black_a">
          <pose>0.328 0.52 0.33 0 0 0</pose>
          <geometry><box><size>0.012 0.25 0.20</size></box></geometry>
          <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>
        </visual>
        <visual name="west_inner_black_b">
          <pose>0.328 0.92 0.70 0 0 0</pose>
          <geometry><box><size>0.012 0.19 0.30</size></box></geometry>
          <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>
        </visual>
        <visual name="west_outer_board">
          <pose>-0.20 2.65 0.55 0 0 0</pose>
          <geometry><box><size>0.05 0.90 0.95</size></box></geometry>
          <material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.19 0.19 0.19 1</diffuse></material>
        </visual>
        <visual name="west_outer_white_a">
          <pose>-0.172 2.43 0.31 0 0 0</pose>
          <geometry><box><size>0.012 0.24 0.18</size></box></geometry>
          <material><ambient>0.98 0.98 0.98 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
        <visual name="west_outer_white_b">
          <pose>-0.172 2.82 0.70 0 0 0</pose>
          <geometry><box><size>0.012 0.26 0.26</size></box></geometry>
          <material><ambient>0.98 0.98 0.98 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>

        <!-- Southbound leg: boards face +Y. -->
        <visual name="south_inner_board">
          <pose>-0.75 0.20 0.55 0 0 0</pose>
          <geometry><box><size>0.95 0.05 0.95</size></box></geometry>
          <material><ambient>0.82 0.82 0.82 1</ambient><diffuse>0.90 0.90 0.90 1</diffuse></material>
        </visual>
        <visual name="south_inner_black_a">
          <pose>-0.98 0.228 0.35 0 0 0</pose>
          <geometry><box><size>0.27 0.012 0.22</size></box></geometry>
          <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>
        </visual>
        <visual name="south_inner_black_b">
          <pose>-0.58 0.228 0.72 0 0 0</pose>
          <geometry><box><size>0.20 0.012 0.28</size></box></geometry>
          <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>
        </visual>
        <visual name="south_outer_board">
          <pose>-2.65 -0.30 0.55 0 0 0</pose>
          <geometry><box><size>0.90 0.05 0.95</size></box></geometry>
          <material><ambient>0.14 0.14 0.14 1</ambient><diffuse>0.18 0.18 0.18 1</diffuse></material>
        </visual>
        <visual name="south_outer_white_a">
          <pose>-2.87 -0.272 0.30 0 0 0</pose>
          <geometry><box><size>0.23 0.012 0.17</size></box></geometry>
          <material><ambient>0.98 0.98 0.98 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
        <visual name="south_outer_white_b">
          <pose>-2.48 -0.272 0.68 0 0 0</pose>
          <geometry><box><size>0.29 0.012 0.25</size></box></geometry>
          <material><ambient>0.98 0.98 0.98 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>

        <!-- Floor markers add depth variation without affecting collision. -->
        <visual name="floor_marker_a">
          <pose>-1.10 -1.35 0.008 0 0 0</pose>
          <geometry><box><size>0.24 0.18 0.012</size></box></geometry>
          <material><ambient>0.03 0.03 0.03 1</ambient><diffuse>0.03 0.03 0.03 1</diffuse></material>
        </visual>
        <visual name="floor_marker_b">
          <pose>1.35 -1.10 0.008 0 0 0.35</pose>
          <geometry><box><size>0.30 0.16 0.012</size></box></geometry>
          <material><ambient>0.94 0.94 0.94 1</ambient><diffuse>0.98 0.98 0.98 1</diffuse></material>
        </visual>
        <visual name="floor_marker_c">
          <pose>1.05 1.35 0.008 0 0 -0.25</pose>
          <geometry><box><size>0.20 0.28 0.012</size></box></geometry>
          <material><ambient>0.03 0.03 0.03 1</ambient><diffuse>0.03 0.03 0.03 1</diffuse></material>
        </visual>
        <visual name="floor_marker_d">
          <pose>-1.35 1.05 0.008 0 0 0.55</pose>
          <geometry><box><size>0.32 0.14 0.012</size></box></geometry>
          <material><ambient>0.94 0.94 0.94 1</ambient><diffuse>0.98 0.98 0.98 1</diffuse></material>
        </visual>
      </link>
    </model>

"""


def create_world(source: Path, destination: Path) -> None:
    text = source.read_text()
    if text.count(INSERT_BEFORE) != 1:
        raise RuntimeError("expected exactly one slam_bot insertion marker")
    generated = text.replace(INSERT_BEFORE, FEATURE_BLOCK + INSERT_BEFORE)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(generated)
    ET.parse(destination)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    create_world(args.source, args.destination)
    print(args.destination)


if __name__ == "__main__":
    main()
