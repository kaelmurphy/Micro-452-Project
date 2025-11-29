# polygons: id -> list of (x, y) points (we'll talk about order next)
polygons = {
    pid: [(float(x), float(y)) for x, y in g[["x","y"]].to_numpy()]
    for pid, g in df.groupby("id")
    if len(g) >= 3
}

# special points we may want later (start/goal/waypoints)
special_points = {
    row["type"].lower(): (float(row["x"]), float(row["y"]))
    for _, row in df[df["type"].str.lower().isin(["start","goal","waypoint"])].iterrows()
}


#Recalculate new nodes
EPSILON_CM = 2.0  # cm, or whatever you want
EPSILON = EPSILON_CM * UNIT_SCALE  # now 20.0 mm


# shapely polygons for inflated world


# nodes for inflated world

# neighbors for inflated world

node_coords = node_coords_eps
node_labels = node_labels_eps
neighbors   = neighbors_eps

#print("=== LIST OF NODES ===")
#for i, (coord, label) in enumerate(zip(node_coords, node_labels)):
#    print(f"Node {i:2d} | label={label:8s} | coord={coord}")

print("\n=== NEIGHBOR COUNTS ===")
for i, nbrs in enumerate(neighbors):
    print(f"Node {i:2d} ({node_labels[i]:8s}) has {len(nbrs)} neighbors")



if path_indices is None:
    print("No path found.")
else:
    print("A* operation count:", op_count)
    print("Number of explored nodes:", len(explored_nodes))
    print("Path node indices:", path_indices)

    # If you have node_labels:
    print("Path labels:", [node_labels[i] for i in path_indices])
    print("Path coordinates:", [node_coords[i] for i in path_indices])