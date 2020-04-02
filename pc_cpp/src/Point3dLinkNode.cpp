//
// Created by bab21 on 24.03.20.
//

#include "../h/Point3dLinkNode.hpp"
#include "Point3d.cpp"


Point3dLinkNode::Point3dLinkNode() {
    last = next = nullptr;
}

Point3dLinkNode::Point3dLinkNode(Point3d *from) {
    x = from->x;
    y = from->y;
    z = from->z;
    last = next = nullptr;
}