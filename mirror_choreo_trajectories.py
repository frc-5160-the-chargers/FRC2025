import json
import os

field_width = 57.573 / 3.281
field_height = 26.417 / 3.281

choreoDirectory = "../src/main/deploy/choreo"

def load_traj(file_path):
    with open(file_path, 'r') as f:
        traj = json.load(f)
    return traj

def mirror_snap_point(point):
    #Mirror a point across the y centerline of the field making sure to rotate accordingly
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

def flipExpression(expression):
    exp = expression["exp"]
    val = expression["val"]

    #add - to the exp value if it is not already there
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
    pYVal = field_height - point["y"]["val"]
    pYExp = str(pYVal) + " m"
    point["y"]["val"] = pYVal
    point["y"]["exp"] = pYExp

    pHeadingVal = -point["heading"]["val"]
    pHeadingExp = str(pHeadingVal) + " rad"

    point["heading"]["val"] = pHeadingVal
    point["heading"]["exp"] = pHeadingExp

    return point

def main():

    choreoFiles = os.listdir(choreoDirectory)

    #Find only .traj files that do not contain the word "mirrored"
    trajFiles = [file for file in choreoFiles if file.endswith(".traj") and "mirrored" not in file]

    for file in trajFiles:
        traj = load_traj(choreoDirectory + "\\" + file)
        snapshot_waypoints = traj["snapshot"]["waypoints"]
        param_waypoints = traj["params"]["waypoints"]

        for i, point in enumerate(snapshot_waypoints):
            snapshot_waypoints[i] = mirror_snap_point(point)

        for i, point in enumerate(param_waypoints):
            param_waypoints[i] = mirror_param_point(point)

        #save the mirrored traj
        with open(choreoDirectory + "\\" + "mirrored_" + file, 'w') as f:
            json.dump(traj, f, indent=4)

if __name__ == '__main__':
    print("hi!!!")
    main()
