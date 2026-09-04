#!/usr/bin/env python3

import struct
import sys
from pathlib import Path


TRIANGLE_SIZE = 50
MAX_COMPONENT_EXTENT_MM = 100.0


def triangle_vertices(record):
    values = struct.unpack("<12fH", record)

    return [
        (values[3], values[4], values[5]),
        (values[6], values[7], values[8]),
        (values[9], values[10], values[11]),
    ]


def component_bbox(indices, triangles):
    minimum = [float("inf")] * 3
    maximum = [float("-inf")] * 3

    for index in indices:
        for vertex in triangle_vertices(triangles[index]):
            for axis in range(3):
                minimum[axis] = min(minimum[axis], vertex[axis])
                maximum[axis] = max(maximum[axis], vertex[axis])

    extent = [
        maximum[axis] - minimum[axis]
        for axis in range(3)
    ]

    return minimum, maximum, extent


def main():
    if len(sys.argv) != 3:
        print("Usage: clean_jt128_mesh.py INPUT.stl OUTPUT.stl")
        return 1

    input_path = Path(sys.argv[1])
    output_path = Path(sys.argv[2])

    data = input_path.read_bytes()

    triangle_count = struct.unpack("<I", data[80:84])[0]
    expected_size = 84 + triangle_count * TRIANGLE_SIZE

    if len(data) != expected_size:
        raise ValueError("The input is not a supported binary STL file")

    triangles = [
        data[
            84 + index * TRIANGLE_SIZE:
            84 + (index + 1) * TRIANGLE_SIZE
        ]
        for index in range(triangle_count)
    ]

    parent = list(range(triangle_count))
    component_size = [1] * triangle_count

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(first, second):
        first = find(first)
        second = find(second)

        if first == second:
            return

        if component_size[first] < component_size[second]:
            first, second = second, first

        parent[second] = first
        component_size[first] += component_size[second]

    vertex_owner = {}

    for triangle_index, triangle in enumerate(triangles):
        for vertex in triangle_vertices(triangle):
            key = tuple(round(coordinate, 5) for coordinate in vertex)

            if key in vertex_owner:
                union(triangle_index, vertex_owner[key])
            else:
                vertex_owner[key] = triangle_index

    components = {}

    for triangle_index in range(triangle_count):
        root = find(triangle_index)
        components.setdefault(root, []).append(triangle_index)

    removed = set()

    for indices in components.values():
        _, _, extent = component_bbox(indices, triangles)

        if max(extent) > MAX_COMPONENT_EXTENT_MM:
            removed.update(indices)
            print(
                f"Removed auxiliary component: "
                f"{len(indices)} triangles, extent={extent}"
            )

    kept_indices = [
        index
        for index in range(triangle_count)
        if index not in removed
    ]

    minimum, maximum, extent = component_bbox(
        kept_indices,
        triangles,
    )

    header = b"JT128 physical geometry".ljust(80, b"\0")

    with output_path.open("wb") as output_file:
        output_file.write(header)
        output_file.write(struct.pack("<I", len(kept_indices)))

        for index in kept_indices:
            output_file.write(triangles[index])

    print(f"Input triangles:  {triangle_count}")
    print(f"Output triangles: {len(kept_indices)}")
    print(f"Physical minimum: {minimum}")
    print(f"Physical maximum: {maximum}")
    print(f"Physical extent:  {extent}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())