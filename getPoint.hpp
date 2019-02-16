#inlude "defs.hpp"

float4 getPoint(int i, int j){
  float4 point3D;
  // Get the 3D point cloud values for pixel (i,j)
  point_cloud.getValue(i,j,&point3D);
  return point3D;
}
