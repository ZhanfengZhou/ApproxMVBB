// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include <iostream>
#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "ApproxMVBB/ComputeApproxMVBB.hpp"
#include "npy.hpp"

int data_load(char * idx, std::vector<double> &x, std::vector<double> &y, std::vector<double> &z) {
  std::vector<unsigned long> shape;
  bool fortran_order;
  std::vector<double> data;
  std::vector<const char*> ap {
    
    "/home/zhanfeng/camera_ws/src/ApproxMVBB/data/input/object_",
    "/home/zhanfeng/camera_ws/src/ApproxMVBB/data/input/object_",
    "/home/zhanfeng/camera_ws/src/ApproxMVBB/data/input/object_",
    
  };
  std::vector<char *> allpaths;
  std::cout<<"Start estimate MVBB of each object:"<<std::endl;

  allpaths.resize(3); 

  for (int i = 0; i < 3; i++) {
    allpaths[i] = (char*)malloc(100 * sizeof(char));
  }
  
  std::strcpy(allpaths[0], ap[0]);
  std::strcat(allpaths[0], idx);
  std::strcat(allpaths[0], "_x_cleaned.npy");

  std::strcpy(allpaths[1], ap[1]);
  std::strcat(allpaths[1], idx);
  std::strcat(allpaths[1], "_y_cleaned.npy");

  std::strcpy(allpaths[2], ap[2]);
  std::strcat(allpaths[2], idx);
  std::strcat(allpaths[2], "_z_cleaned.npy");

  
  int count = 0;

  std::cout<<"Reading point-cloud 3D file for each object"<<std::endl;
  for (auto path : allpaths) {
    shape.clear();
    data.clear();
    npy::LoadArrayFromNumpy(path, shape, fortran_order, data);
    
    if (count == 0) {
      x = data;
    } else if (count == 1) {
      y = data;
    } else if (count == 2) {
      z = data;
    }
    count++;
  }

  std::cout<<"Read finished"<<std::endl;

  for (int i = 0; i < 3; i++) {
    free(allpaths[i]);
  }


  return 0;
}

int data_save(std::vector<double> &data, std::string filename) {
  std::array<long unsigned, 1> datashape {{data.size()}};
  npy::SaveArrayAsNumpy(filename, false, datashape.size(), datashape.data(), data);

  return 0;
}

int mvbb(char * idx, std::vector<double> vec_x, std::vector<double> vec_y, std::vector<double> vec_z) {
    int pc_len = vec_x.size();

    
    unsigned int nPoints = pc_len;
    std::cout << "Sample " << nPoints << " points in unite cube (coordinates are in world coordinate system `I` ) " << std::endl;
    ApproxMVBB::Matrix3Dyn points(3, nPoints);
    
    for(unsigned int i = 0; i < nPoints; i++) {
      points(0,i) = vec_x[i];
      points(1,i) = vec_y[i];
      points(2,i) = vec_z[i];
    }

    ApproxMVBB::OOBB oobb = ApproxMVBB::approximateMVBB(points,
                                                        0.001,
                                                        500,
                                                        5, /*increasing the grid size decreases speed */
                                                        0,
                                                        5);

    std::cout << "Computed OOBB: " << std::endl
              << "---> lower point in OOBB coordinate system: " << oobb.m_minPoint.transpose() << std::endl
              << "---> upper point in OOBB coordinate system: " << oobb.m_maxPoint.transpose() << std::endl
              << "---> coordinate transformation A_IK matrix from OOBB coordinate system `K`  "
                 "to world coordinate system `I` "
              << std::endl
              << oobb.m_q_KI.matrix() << std::endl
              << "---> this is also the rotation matrix R_KI  which turns the "
                 "world coordinate system `I`  into the OOBB coordinate system `K` "
              << std::endl
              << std::endl;

    std::vector<double> minpt = {oobb.m_minPoint(0), oobb.m_minPoint(1), oobb.m_minPoint(2)};
    std::vector<double> maxpt = {oobb.m_maxPoint(0), oobb.m_maxPoint(1), oobb.m_maxPoint(2)};
    std::vector<double> trans_matrix_ln0 = {oobb.m_q_KI.matrix()(0, 0), oobb.m_q_KI.matrix()(0, 1), oobb.m_q_KI.matrix()(0, 2)};
    std::vector<double> trans_matrix_ln1 = {oobb.m_q_KI.matrix()(1, 0), oobb.m_q_KI.matrix()(1, 1), oobb.m_q_KI.matrix()(1, 2)};
    std::vector<double> trans_matrix_ln2 = {oobb.m_q_KI.matrix()(2, 0), oobb.m_q_KI.matrix()(2, 1), oobb.m_q_KI.matrix()(2, 2)};

    char f0[100];
    char f1[100];
    char f2[100];
    char f3[100];
    char f4[100];
    
    std::cout<<"MVBB finished, start saving results"<<std::endl;

    std::strcpy(f0, "/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/m_minPoint");
    std::strcpy(f1, "/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/m_maxPoint");
    std::strcpy(f2, "/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/trans_matrix_");
    std::strcpy(f3, "/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/trans_matrix_");
    std::strcpy(f4, "/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/trans_matrix_");

    std::strcat(f0, idx);
    std::strcat(f0, ".npy");

    std::strcat(f1, idx);
    std::strcat(f1, ".npy");

    std::strcat(f2, idx);
    std::strcat(f2, "x.npy");

    std::strcat(f3, idx);
    std::strcat(f3, "y.npy");

    std::strcat(f4, idx);
    std::strcat(f4, "z.npy");

    std::cout<<"Saving results"<<std::endl;
    

    data_save(minpt, f0);
    data_save(maxpt, f1);
    data_save(trans_matrix_ln0, f2);
    data_save(trans_matrix_ln1, f3);
    data_save(trans_matrix_ln2, f4);

    std::cout<<"Saved, all done"<<std::endl;

    return 0;
}


int main(int argc, char** argv)
{
    std::vector<double> vec_x;
    std::vector<double> vec_y;
    std::vector<double> vec_z;

    char idx[] = "0";

    int num_of_objects = 6;

    for (int i = 0; i < num_of_objects; i++) {
      idx[0] = (char)(i + 48); 
      data_load(idx, vec_x, vec_y, vec_z);
      mvbb(idx, vec_x, vec_y, vec_z);
    }
    
    return 0;
}


