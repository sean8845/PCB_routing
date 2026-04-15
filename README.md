# **PCB\_routing**

## Basic Astar

### Execution Workflow
The routing process is divided into two main stages: clearing the board and generating new routes.



#### 1\. Clear Existing Tracks (Rip-up)

- Before running the routing algorithm, you must remove any existing tracks or vias to create a clean workspace.
- Script: basic\_Astar/ripup.py
- Action: This script reads your original .kicad\_pcb file, strips away all pre-existing routing data, and saves a clean version to be used as the input for the router.



#### 2\. Execute A\* Routing Logic

- Once the board is clear, you can run the pathfinding algorithm to connect the netlist.
- Available Scripts:

    a. routing\_4direction.py: Restricts routing to horizontal and vertical movements only.

    b. routing\_8direction.py: Allows diagonal movements for more efficient and flexible paths.
    
- Action: The script calculates the shortest path between pins based on grid coordinates and writes the new track data back into a KiCad-compatible file.

