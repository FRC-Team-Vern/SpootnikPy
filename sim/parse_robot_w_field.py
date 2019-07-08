import xml.etree.ElementTree as ET
import numpy as np


def extractPathsFromSVG(input_filename):
    tree = ET.parse(input_filename)
    root = tree.getroot()
    viewBox = root.attrib['viewBox'].split()
    field_width = float(root.attrib['width'])
    field_height = float(root.attrib['height'])
    view_width = float(viewBox[2])
    view_height = float(viewBox[3])
    width_ratio = field_width / view_width
    height_ratio = field_height / view_height
    print(f"field width: {field_width:<10} field height: {field_height:<10} width ratio: {width_ratio:<10.04f} height ratio: {height_ratio:<10.04f}")
    # prev_pt = (0, 0)

    g = root.find('{http://www.w3.org/2000/svg}g')

    if g:
        transform = [float(x) for x in g.attrib['transform'].replace('translate(', '').replace(')', '').split(',')]
    else:
        transform = (0, 0)

    paths_as_points = {}
    for child in root.iter('{http://www.w3.org/2000/svg}path'):
        id, d, style = [child.attrib[key] for key in ['id', 'd', 'style']]

        _, _, color = style.split(";")[0].partition(":")
        # prev_pt, object_pts = parse_pts(d, prev_pt, width_ratio, height_ratio, transform)
        object_pts = parse_pts(d, width_ratio, height_ratio, transform)
        # if id == 'left_wheel':
        #     rounded_object_pts = [(round(pt[0], 3), round(pt[1], 3)) for pt in object_pts]
        #     print(f"left_wheel pts: {rounded_object_pts}")
        paths_as_points[id] = (color, [(round(pt[0], 3), round(pt[1], 3)) for pt in object_pts])

    return paths_as_points


def parse_pts(d, width_ratio, height_ratio, transform):
    pts = []
    d_split = d.split()
    i = 0
    prev_move_type = None
    first_pt = None
    prev_pt = (0, 0)
    prev_x, prev_y = prev_pt
    while i < len(d_split):
        move_type = d_split[i]
        x, y = [None] * 2
        prev_x, prev_y = prev_pt
        i += 1
        if move_type == "M":
            x, y = [float(t) for t in d_split[i].split(',')]
        elif move_type == "m":
            x, y = [float(t) for t in d_split[i].split(',')]
            # dx, dy = [float(t) for t in d_split[i].split(',')]
            # x, y = (prev_x / width_ratio) - transform[0] + dx, (prev_y / height_ratio) - transform[1] + dy
        elif move_type == "H":
            prev_y = (prev_y / height_ratio) - transform[1]
            x, y = float(d_split[i]), prev_y
        elif move_type == "h":
            prev_x, prev_y = (prev_x / width_ratio) - transform[0], (prev_y / height_ratio) - transform[1]
            dx = float(d_split[i])
            x, y = prev_x + dx, prev_y
        elif move_type == "V":
            prev_x = (prev_x / width_ratio) - transform[0]
            x, y = prev_x, float(d_split[i])
        elif move_type == "v":
            prev_x, prev_y = (prev_x / width_ratio) - transform[0], (prev_y / height_ratio) - transform[1]
            dy = float(d_split[i])
            x, y = prev_x, prev_y + dy
        elif move_type.upper() == "Z":
            # We don't need to close paths in tkinter, but we do need to pass them on for relative moves in SVG
            x, y = (first_pt[0] / width_ratio) - transform[0], (first_pt[1] / height_ratio) - transform[1]
        else:
            i -= 1
            coords = [float(t) for t in d_split[i].split(',')]
            if len(coords) == 2:
                if prev_move_type == "M":
                    x, y = coords
                    move_type = prev_move_type
                elif prev_move_type == "m":
                    dx, dy = coords
                    x, y = (prev_x / width_ratio) - transform[0] + dx, (prev_y / height_ratio) - transform[1] + dy
                    move_type = prev_move_type
                else:
                    raise ValueError(f"unknown move type: {move_type}")
            else:
                raise ValueError(f"unknown move type: {move_type}")
        i += 1

        x, y = (x + transform[0]) * width_ratio, (y + transform[1]) * height_ratio
        pt = (x, y)
        prev_pt = pt

        if move_type.upper() == "M":
            first_pt = pt

        if move_type.upper() != "Z":
            pts.append(pt)

        prev_move_type = move_type

    return pts


def convertExtractPtsToPx(extracted_pts, px_to_ft_conv):
    scaled_extracted_pts = {}
    for id, extracted in extracted_pts.items():
        color, pts = extracted
        new_extracted = [(round(pt[0] * px_to_ft_conv, 3), round(pt[1] * px_to_ft_conv, 3)) for pt in pts]
        scaled_extracted_pts[id] = (color, new_extracted)

    return scaled_extracted_pts


def pathsPtsAsObjectJson(path_pts):
    # {"color": "#888", "points": [(0, 15), (40, 5)],
    #  {"color": "blue", "points": [(0, 15), (40, 5)]
    json_strs = []
    for id, path_info in path_pts.items():
        color, pts = path_info
        centroid_x, centroid_y = calcCentroid(pts)
        pts_str = "[" + ", ".join([f"[{pt[0]}, {pt[1]}]" for pt in pts]) + "]"
        json_str = f'{{"color": "{color}", "center": [{centroid_x:.03f}, {centroid_y:.03f}], "points": {pts_str}}}'
        json_strs.append(json_str)

    return json_strs


def calcCentroid(pts):
    arr = np.array(pts)
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x / length, sum_y / length


if __name__ == '__main__':
    # input_filename = 'robot_w_field_plain.svg'
    input_filename = '../test/robot_w_field_plain.svg'

    extracted_pts = extractPathsFromSVG(input_filename)

    # Final points need to be in "feet" units.
    px_to_ft_conv = 0.1
    extracted_pts = convertExtractPtsToPx(extracted_pts, px_to_ft_conv)

    object_paths_as_json = pathsPtsAsObjectJson(extracted_pts)

    json_str = ",\n".join(object_paths_as_json)
    print(json_str)
