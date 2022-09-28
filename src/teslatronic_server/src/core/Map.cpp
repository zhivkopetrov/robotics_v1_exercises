#include "teslatronic_server/core/Map.h"

namespace {
  constexpr int8_t EMPTY_TILE_SYMBOL = '.';
}

int32_t Map::init(int32_t rows, int32_t cols) {
  _data.resize(rows * cols, EMPTY_TILE_SYMBOL);

  return EXIT_SUCCESS;
}
