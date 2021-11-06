#!/usr/bin/env python

# STL
import json
import numpy as np
import argparse
from collections import OrderedDict

# ROS
from tf.transformations import quaternion_from_matrix, quaternion_matrix

def compute_cog(array_3d):

  x_sum = 0.0
  y_sum = 0.0
  for idx in range(len(array_3d.shape[0])):
    array_3d[idx][0]

def solve_rigid_transform(from_matrix, to_matrix):

  # X. Compute cog
  from_cog = np.mean(from_matrix, axis=0)
  to_cog = np.mean(to_matrix, axis=0)

  # X. Relative to cog
  from_pnts_from_cog = from_matrix - from_cog
  to_pnts_from_cog = to_matrix - to_cog

  # X. Solve rotation
  m = np.matmul(np.transpose(from_pnts_from_cog), to_pnts_from_cog)
  u, s, vh = np.linalg.svd(m, full_matrices=True)
  v = np.transpose(vh)
  uh = np.transpose(u)
  
  R = np.matmul(v, uh)
  if (np.linalg.det(R) < 0):
    v[:,2] = -1 * v[:,2]
    R = np.matmul(v, uh)

  R4 = np.identity(4, dtype=np.float64)
  R4[:3, :3] = R

  q = quaternion_from_matrix(R4)

  return from_cog, to_cog, q

def create_numpy_matrix_from_json(paired_pose_json):

  size = len(paired_pose)
  gnss_pnts = np.zeros((size, 3), dtype=np.float64)
  loc_pnts = np.zeros((size, 3), dtype=np.float64)

  for idx in range(size):
    pair = paired_pose[idx]
    gnss_pnts[idx, 0] = pair['gnss']['x']
    gnss_pnts[idx, 1] = pair['gnss']['y']
    gnss_pnts[idx, 2] = pair['gnss']['z']

    loc_pnts[idx, 0] = pair['localized']['x']
    loc_pnts[idx, 1] = pair['localized']['y']
    loc_pnts[idx, 2] = pair['localized']['z']

  return gnss_pnts, loc_pnts

def verify_result(from_matrix, to_matrix, from_cog, to_cog, q):

  # X. Apply from_cog
  from_matrix_from_cog = from_matrix - from_cog

  # X. Apply rotation.
  R = quaternion_matrix(q)[:3,:3]
  from_matrix_from_cog_rotated = np.transpose(np.matmul(R, np.transpose(from_matrix_from_cog)))

  # X. Apply to_cog
  from_matrix_transformed = from_matrix_from_cog_rotated + to_cog

  # X. Compute residuals
  diff_matrix = to_matrix - from_matrix_transformed
  residuals = np.linalg.norm(diff_matrix, axis=1)

  print('Maximum residuals : {}'.format(np.max(residuals)))
  print('Average residuals : {}'.format(np.mean(residuals)))


if __name__ == '__main__':

  parser = argparse.ArgumentParser()
  parser.add_argument('--pose_pair_filepath', type=str, required=True)
  parser.add_argument('--transform_filepath', type=str, required=True)
  args = parser.parse_args()

  with open(args.pose_pair_filepath, 'r') as f:
    paired_pose = json.load(f)

  # X. Parse json file into numpy array.
  gnss_pnts, loc_pnts = create_numpy_matrix_from_json(paired_pose)
  
  # X. Solve transform.
  gnss_cog, loc_cog, q = solve_rigid_transform(gnss_pnts, loc_pnts)

  # X. Verify results.
  verify_result(gnss_pnts, loc_pnts, gnss_cog, loc_cog, q)

  gnss_cog = OrderedDict([('x',gnss_cog[0]), ('y',gnss_cog[1]), ('z',gnss_cog[2])])
  quat = OrderedDict([('x',q[0]), ('y',q[1]), ('z',q[2]), ('w',q[3])])
  map_cog = OrderedDict([('x',loc_cog[0]), ('y',loc_cog[1]), ('z',loc_cog[2])])

  # X. Save to json file.
  with open(args.transform_filepath, 'w') as f:
    json.dump(OrderedDict([('gnss_cog', gnss_cog), ('quat', quat), ('map_cog', map_cog)]), f, indent=4)
  
