#ifndef MAP_H_
#define MAP_H_

#include "teslatronic_server/teslatronic_common/CommonDefines.h"

class Map {
public:
  int32_t init(int32_t rows, int32_t cols);

  const MapDescription& getMapDescr() const;

private:
  MapDescription _mapDescr;
};

#endif /* MAP_H_ */
