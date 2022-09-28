#ifndef MAP_H_
#define MAP_H_

#include <vector>

class Map {
public:
  int32_t init(int32_t rows, int32_t cols);

private:
  std::vector<int8_t> _data;
};

#endif /* MAP_H_ */
