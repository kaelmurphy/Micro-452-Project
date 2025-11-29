from vision import getVisionCoords

if __name__ == "__main__":
    # Call the vision system and print the output
    coordinates, theta = getVisionCoords(timeout=None, showDisplay=True)
    print("Returned coordinates array:")
    print(coordinates, "radians: {:.2f}".format(theta))