/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2016 Jose Luis Blanco (joseluisblancoc@gmail.com).
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/
#ifndef _UTILS_H_
#define _UTILS_H_
#pragma once

#include <cstdlib>
#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

template <typename T>
struct PointCloud
{
	struct PtData
	{
		T x{0};
		T y{0};
		T z{0};
		float intensity{0.0f};
		uint8_t r{0};
		uint8_t g{0};
		uint8_t b{0};

		PtData(T xx, T yy, T zz) {
		  x = xx;
		  y = yy;
		  z = zz;
	    }

        PtData(T in_x, T in_y, T in_z,
               uint8_t in_r, uint8_t in_g, uint8_t in_b) {
		  x = in_x;
		  y = in_y;
		  z = in_z;
		  r = in_r;
		  g = in_g;
		  b = in_b;
	    }

		PtData &operator =(const PtData &other)
		{
			this->x = other.x;
			this->y = other.y;
			this->z = other.z;
			this->intensity = other.intensity;
			this->r = other.r;
			this->g = other.g;
			this->b = other.b;
			return *this;
		}
	};

	std::vector<PtData>  pts;

	// operator =
	PointCloud &operator =(const PointCloud &info)
	{
		this->pts    = info.pts;
		return *this;
	}

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline T kdtree_get_pt(const size_t idx, int dim) const
	{
		if (dim == 0) return pts[idx].x;
		else if (dim == 1) return pts[idx].y;
		else return pts[idx].z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

    void SaveToXYZPlyFile(const std::string &file_path) const {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      cloud.is_dense = false;
      cloud.resize(this->pts.size());
      for (size_t i = 0; i < this->pts.size(); ++i) {
        const auto& p = this->pts[i];
        auto& p_out = cloud[i];
        p_out.x = p.x;
        p_out.y = p.y;
        p_out.z = p.z;
      }

      pcl::PLYWriter ply_writer;
      ply_writer.write(file_path, cloud);

      std::cout << "PointCloud::SaveToXYZPlyFile(): n="
                << cloud.size() << std::endl;
    }

    void ReadFromXYZPlyFile(const std::string &file_path) {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::io::loadPLYFile(file_path, cloud);

      this->pts.clear();
      this->pts.reserve(cloud.size());
      for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& p_in = cloud[i];
        this->pts.emplace_back(p_in.x, p_in.y, p_in.z);
      }

      std::cout << "PointCloud::ReadFromXYZPlyFile(): n="
                << this->pts.size() << std::endl;
    }

    void SaveToXYZRGBPlyFile(const std::string &file_path) const {
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      cloud.is_dense = false;
      cloud.resize(this->pts.size());
      for (size_t i = 0; i < this->pts.size(); ++i) {
        const auto& p = this->pts[i];
        auto& p_out = cloud[i];
        p_out.x = p.x;
        p_out.y = p.y;
        p_out.z = p.z;
        p_out.r = p.r;
        p_out.g = p.g;
        p_out.b = p.b;
      }

      pcl::PLYWriter ply_writer;
      ply_writer.write(file_path, cloud);

      std::cout << "PointCloud::SaveToXYZRGBPlyFile(): n="
                << cloud.size() << std::endl;
    }

    void ReadFromXYZRGBPlyFile(const std::string &file_path) {
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      pcl::io::loadPLYFile(file_path, cloud);

      this->pts.clear();
      this->pts.reserve(cloud.size());
      for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& p_in = cloud[i];
        this->pts.emplace_back(p_in.x, p_in.y, p_in.z,
                               p_in.r, p_in.g, p_in.b);
      }

      std::cout << "PointCloud::ReadFromXYZRGBPlyFile(): n="
                << this->pts.size() << std::endl;
    }
};

#endif //