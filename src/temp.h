#pragma once
#include "dataStructure/Path.h"

Path generateCirclePath(float centerX, float centerZ, float height, float radius, int segments, float lineWidth);
Path generateZigzagFillInCircle(float centerX, float centerZ, float height, float radius, float step, float lineWidth);