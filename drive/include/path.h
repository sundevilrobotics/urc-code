#ifndef PATH_H
#define PATH_H

#include waypoint.h
#include map.h

class PathNode
{
private:
  PathNode next;
  float position;

public:
  PathNode();
}


class Path
{
private:
  PathNode* list;
  Waypoint start;
  Waypoint end;
public:
  Path(Waypoint &start, Waypoint &end)
  {
    list = NULL;
    this->start = start;
    this->end = end;
  };
  PathNode* calculatePath()
  {



    list = new PathNode();

  };
};

#endif
