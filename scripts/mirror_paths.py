import json
import os

field_width = 57.573 / 3.281
field_height = 26.417 / 3.281

choreo_dir = "../src/main/deploy/choreo"

def load_traj(file_path):
    with open(file_path, 'r') as f:
        traj = json.load(f)
    return traj

def mirror_snap_point(point):
    # Mirror a point across the y centerline of the field making sure to rotate accordingly
    point = {
        "x": point["x"],
        "y": field_height - point["y"],
        "heading": -point["heading"],
        "intervals": point["intervals"],
        "split": point["split"],
        "fixTranslation": point["fixTranslation"],
        "fixHeading": point["fixHeading"],
        "overrideIntervals": point["overrideIntervals"]
    }
    return point

def flip_expression(expression):
    exp = expression["exp"]
    val = expression["val"]

    # add - to the exp value if it is not already there
    if exp[0] != "-":
        exp = "-" + exp
    else:
        exp = exp[1:]

    val = -val

    return {
        "exp": exp,
        "val": val
    }

def mirror_param_point(point):
    p_y_val = field_height - point["y"]["val"]
    p_y_exp = str(p_y_val) + " m"
    point["y"]["val"] = p_y_val
    point["y"]["exp"] = p_y_exp

    p_heading_val = -point["heading"]["val"]
    p_heading_exp = str(p_heading_val) + " rad"

    point["heading"]["val"] = p_heading_val
    point["heading"]["exp"] = p_heading_exp

    return point

def main():
    choreo_files = os.listdir(choreo_dir)

    # Find only .traj files that do not contain the word "mirrored"
    traj_files = [file for file in choreo_files if file.endswith(".traj") and "mirrored" not in file]

    for file in traj_files:
        traj = load_traj(choreo_dir + "\\" + file)
        snapshot_waypoints = traj["snapshot"]["waypoints"]
        param_waypoints = traj["params"]["waypoints"]

        for i, point in enumerate(snapshot_waypoints):
            snapshot_waypoints[i] = mirror_snap_point(point)

        for i, point in enumerate(param_waypoints):
            param_waypoints[i] = mirror_param_point(point)

        # save the mirrored traj
        with open(choreo_dir + "\\" + "mirrored_" + file, 'w') as f:
            json.dump(traj, f, indent=4)

if __name__ == '__main__':
    main()