#ifndef _LINE_DETECTION_H_
#define _LINE_DETECTION_H_
#pragma once

#include <string>

#include "CommonFunctions.h"

typedef std::vector<cv::Point3d> Line3D;
typedef std::vector<Line3D> Line3DList;

struct PLANE
{
    double scale;
    std::vector<Line3DList> lines3d;

    PLANE &operator =(const PLANE &info)
    {
        this->scale    = info.scale;
        this->lines3d     = info.lines3d;
        return *this;
    }
};
typedef std::vector<PLANE> PlaneList;


class LineDetection3D 
{
 public:
    LineDetection3D();
    ~LineDetection3D();

    void run(PointCloud<double> &data, int k, PlaneList &planes,
             Line3DList &lines, std::vector<double> &ts);

    void pointCloudSegmentation( std::vector<std::vector<int> > &regions );

    void planeBased3DLineDetection(std::vector<std::vector<int> > &regions, PlaneList &planes);

    void postProcessing(PlaneList &planes, Line3DList &lines);

 private:
    void regionGrow( double thAngle, std::vector<std::vector<int> > &regions );

    void regionMerging( double thAngle, std::vector<std::vector<int> > &regions );

    bool maskFromPoint( std::vector<cv::Point2d> &pts2d, double radius, double &xmin, double &ymin, double &xmax, double &ymax, int &margin, cv::Mat &mask );

    void lineFromMask( cv::Mat &mask, int thLineLengthPixel, std::vector<std::vector<std::vector<cv::Point2d> > > &lines );

    void outliersRemoval(PlaneList &planes);

    void lineMerging(PlaneList &planes, Line3DList &lines);

public:
    int k;
    int pointNum;
    double scale, magnitd;
    std::vector<PCAInfo> pcaInfos;
    PointCloud<double> pointData;
};

void WritePlanesToPLYFile(const std::string &file_path,
                          const PlaneList &planes, double scale);

void WriteLinesToPLYFile(const std::string &file_path,
                         const Line3DList &lines, double scale);

#endif //_LINE_DETECTION_H_
