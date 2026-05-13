#ifndef __BOARD_MAP_H__
#define __BOARD_MAP_H__

#include <stdint.h>

/* 坐标点数量 */
#define BOARD_X_POINT_COUNT 5U
#define BOARD_Y_POINT_COUNT 5U

/* 格位数量：左待命区5 + 中间3x3博弈区9 + 右待命区5 = 19 */
#define BOARD_CELL_COUNT 19U

/* 格位映射：cell_id -> x/y索引 */
typedef struct
{
  uint8_t x_idx; /* 0~4 */
  uint8_t y_idx; /* 0~4 */
} BoardCellMap_t;

/* X/Y 轴坐标表（单位：步数，后续按实测标定值填写） */
extern const int32_t board_x_step[BOARD_X_POINT_COUNT];
extern const int32_t board_y_step[BOARD_Y_POINT_COUNT];

/* 19个格位的索引映射表 */
extern const BoardCellMap_t board_cell_map[BOARD_CELL_COUNT];

/* 获取某个格位的实际 X 和 Y 坐标步数 */
void BoardMap_GetCellStep(uint8_t cell_id, int32_t *x_step, int32_t *y_step);

#endif /* __BOARD_MAP_H__ */
