#!/usr/bin/python3

import json
import sys

from PIL import Image
import numpy as np
from ruamel.yaml import YAML

MODEL_RESOLUTIONS = {
    "Pandar40P": (1800, 40),
    "Pandar64": (1800, 64),
    "PandarXT16": (2000, 16),
    "PandarXT32": (2000, 32),
    "PandarXT32M": (2000, 32),
    "PandarAT128": (1200, 128),
    "PandarQT64": (600, 64),
    "PandarQT128": (900, 128),
    "Pandar128E4X": (3600, 128),
}


def load_yaml(yaml_path):
    with open(yaml_path, "r") as file:
        yaml = YAML()
        data = yaml.load(file)

    try:
        filters = data["/**"]["ros__parameters"]["point_filters"]
        filters_dict: dict = json.loads(filters)  # Convert string representation to dictionary
        return filters_dict["ring_section_filter"]
    except (KeyError, SyntaxError):
        print("Invalid YAML structure or missing 'ring_section_filter'.")
        sys.exit(1)


def generate_mask(ring_section_filter, sensor_model):
    width, height = MODEL_RESOLUTIONS[sensor_model]

    # Start with a white image (all points enabled)
    mask = np.full((height, width), 255, dtype=np.uint8)

    def azimuth_to_pixel(azimuth):
        return int(width * azimuth / 360)

    for ring, azi_start, azi_end in ring_section_filter:
        if ring < 0 or ring >= height:
            raise ValueError(f"Invalid ring: {ring}")

        if azi_end < azi_start:
            mask[ring, azimuth_to_pixel(azi_start) : width] = 0  # Set pixels to black
            mask[ring, 0 : azimuth_to_pixel(azi_end)] = 0  # Set pixels to black
        else:
            mask[ring, azimuth_to_pixel(azi_start) : azimuth_to_pixel(azi_end)] = (
                0  # Set pixels to black
            )

    return mask


def save_png(mask, output_path):
    img = Image.fromarray(mask, mode="L")  # 'L' mode for grayscale
    img.save(output_path)


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <sensor_model> <input_yaml>")
        sys.exit(1)

    sensor_model = sys.argv[1]

    if sensor_model not in MODEL_RESOLUTIONS:
        print(f"Invalid sensor model: {sensor_model}")
        print(f"Valid models are: {', '.join(MODEL_RESOLUTIONS.keys())}")
        sys.exit(1)

    yaml_path = sys.argv[2]
    output_path = yaml_path.rsplit(".", 1)[0] + ".png"

    ring_section_filter = load_yaml(yaml_path)
    mask = generate_mask(ring_section_filter, sensor_model)
    save_png(mask, output_path)

    print(f"Mask saved to {output_path}")


if __name__ == "__main__":
    main()
