from vision_interface import getCoordinatesFromVision

if __name__ == "__main__":
    # Call the vision system and print the output
    coordinates = getCoordinatesFromVision(timeout=15, stabilityFrames=15, show_display=True)
    print("Returned coordinates array:")
    print(coordinates)
