namespace noether_edge_paths
{

// Set Input Mesh (raw)
// Set Input Cloud (raw)
// Set Input Mesh (already PCAd)
// Set Input Cloud (already PCAd)

// Set Numerical Parameters
  // Search Radius / Number of Neighbors
  // Where to break one edge from the next
  // Path Standoff
  // Exterior Angle
  // Interior Angle
  // Waypoint Spacing

// If necessary, do the PCA

// Calculate Paths
  // Evaluate the PCA results to identify edges
  // Extract the edges from the mesh/cloud
  // Identify the external edge (longest / most points?)
  // Separate the edge segments somehow (convex decomposition?)
  // Fit B-Splines to each edge group
  // Place evenly spaced waypoints along each B-Spline
  // Rotate the b-spline points, then apply standoff

} // end namespace noether_edge_paths
