from vision_interface import getCoordinatesFromVision

if __name__ == "__main__":
    # Call the vision system and print the output
    coordinates = getCoordinatesFromVision(timeout=15, stabilityFrames=60, show_display=False)
    print("Returned coordinates array:")
    print(coordinates)
