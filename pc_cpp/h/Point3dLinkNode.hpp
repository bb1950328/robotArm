//
// Created by bab21 on 24.03.20.
//

#ifndef ROBOTARM_POINT3DLINKNODE_H
#define ROBOTARM_POINT3DLINKNODE_H


#include "Point3d.hpp"

class Point3dLinkNode : public Point3d {
public:
    Point3dLinkNode *last, *next;

    Point3dLinkNode();

    explicit Point3dLinkNode(Point3d *from);
};


#endif //ROBOTARM_POINT3DLINKNODE_H
