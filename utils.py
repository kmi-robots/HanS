import csv


def load_camera_intrinsics_txt(path_to_intr):
    """
    Expects 3x3 intrinsics matrix as singlespace-separated txt
    See ./data/camera_intrinsics.txt for expected format
    """
    intrinsics = []
    with open(path_to_intr) as f:
        reader = csv.reader(f, delimiter=' ')
        for row in reader:
            if row==[]: continue
            for cell in row:
                if cell=='': continue
                try:
                    intrinsics.append(float(cell.split("  ")[1]))
                except IndexError:
                    try:
                        intrinsics.append(float(cell.split(" ")[1]))
                    except IndexError:
                        intrinsics.append(float(cell))
    return intrinsics


